#!/usr/bin/env python3

################################################################################
#                                                                              #
#  MCAP to JSON Converter for ROS2                                             #
#                                                                              #
#  Copyright (c) 2024 Milosch Meriac <milosch@meriac.com>                      #
#                                                                              #
#  Licensed under the Apache License, Version 2.0 (the "License");             #
#  you may not use this file except in compliance with the License.            #
#  You may obtain a copy of the License at                                     #
#                                                                              #
#      http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                              #
#  Unless required by applicable law or agreed to in writing, software         #
#  distributed under the License is distributed on an "AS IS" BASIS,           #
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#  See the License for the specific language governing permissions and         #
#  limitations under the License.                                              #
#                                                                              #
#  This file was written with the assistance of Claude.ai                      #
#                                                                              #
################################################################################

import argparse
import json
import sys
import base64
import bz2
import os
import glob
import math
from mcap import reader as mcap_reader
from mcap_ros2.decoder import DecoderFactory
from collections import defaultdict
import struct
import re
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False


# CDR type decoders lookup table
CDR_TYPE_DECODERS = {
    "uint8":   {"format": "B", "size": 1, "align": 1},
    "int8":    {"format": "b", "size": 1, "align": 1},
    "uint16":  {"format": "<H", "size": 2, "align": 2},
    "int16":   {"format": "<h", "size": 2, "align": 2},
    "uint32":  {"format": "<I", "size": 4, "align": 4},
    "int32":   {"format": "<i", "size": 4, "align": 4},
    "uint64":  {"format": "<Q", "size": 8, "align": 8},
    "int64":   {"format": "<q", "size": 8, "align": 8},
    "float":   {"format": "<f", "size": 4, "align": 4},
    "double":  {"format": "<d", "size": 8, "align": 8},
    "boolean": {"format": "?", "size": 1, "align": 1},
}


def parse_idl_type(idl_text, type_name):
    """Parse IDL to extract field information for a message type."""
    fields = []

    # Extract just the message name from the full type
    message_name = type_name.split("/")[-1]

    # More flexible struct search that handles nested modules
    # Look for the struct definition anywhere in the IDL
    struct_pattern = rf'struct\s+{message_name}\s*{{([^{{}}]+(?:{{[^{{}}]*}}[^{{}}]*)*)}}'

    struct_match = re.search(struct_pattern, idl_text, re.DOTALL)

    if struct_match:
        struct_body = struct_match.group(1)

        # Parse fields - handle various formats
        # Match: type name; or type name = value; or type::qualified name;
        field_pattern = r'((?:\w+::)*\w+(?:\[\d*\])?)\s+(\w+)(?:\s*=\s*[^;]+)?;'

        for field_match in re.finditer(field_pattern, struct_body):
            field_type = field_match.group(1)
            field_name = field_match.group(2)

            # Clean up type names
            field_type = field_type.replace('::', '_')  # Replace namespace separators
            field_type = field_type.replace('[]', '_array')  # Handle arrays

            # Map common ROS2 types
            type_mapping = {
                'std_msgs_Header': 'Header',
                'builtin_interfaces_Time': 'Time',
                'octet': 'uint8',
            }

            field_type = type_mapping.get(field_type, field_type)
            fields.append((field_type, field_name))

    return fields


def decode_cdr_message(data, fields, idl_text=None, idl_cache=None):
    """Decode CDR message data using field information."""
    result = {}
    offset = 4  # Skip CDR header (4 bytes)

    # Build a map of all struct definitions in the current IDL
    struct_map = {}
    if idl_text:
        # Find all struct definitions in the IDL
        struct_pattern = r'struct\s+(\w+)\s*\{([^{}]+(?:\{[^{}]*\}[^{}]*)*)\}'
        for match in re.finditer(struct_pattern, idl_text, re.DOTALL):
            struct_name = match.group(1)
            struct_body = match.group(2)
            # Parse the fields of this struct
            field_pattern = r'((?:\w+::)*\w+(?:\[\d*\])?)\s+(\w+)(?:\s*=\s*[^;]+)?;'
            struct_fields = []
            for field_match in re.finditer(field_pattern, struct_body):
                field_type = field_match.group(1)
                field_name = field_match.group(2)
                # Clean up type names
                field_type = field_type.replace('::', '_')
                field_type = field_type.replace('[]', '_array')
                # Map common ROS2 types
                type_mapping = {
                    'std_msgs_Header': 'Header',
                    'builtin_interfaces_Time': 'Time',
                    'octet': 'uint8',
                }
                field_type = type_mapping.get(field_type, field_type)
                struct_fields.append((field_type, field_name))
            struct_map[struct_name] = struct_fields

    for field_type, field_name in fields:
        if offset >= len(data):
            break

        try:
            if field_type in CDR_TYPE_DECODERS:
                # Use lookup table for basic types
                decoder = CDR_TYPE_DECODERS[field_type]
                # Align offset
                offset = (offset + decoder["align"] - 1) & ~(decoder["align"] - 1)
                value = struct.unpack_from(decoder["format"], data, offset)[0]
                offset += decoder["size"]
            elif field_type == "string":
                # String is length-prefixed
                offset = (offset + 3) & ~3  # Align to 4 bytes
                length = struct.unpack_from('<I', data, offset)[0]
                offset += 4
                value = data[offset:offset + length].decode('utf-8', errors='replace')
                offset += length
            elif field_type == "Header" or field_type.endswith("Header"):
                # Handle standard ROS2 header
                offset = (offset + 3) & ~3  # Align to 4 bytes
                # Header has stamp (Time) and frame_id (string)
                header = {}
                # Time stamp - sec (int32) + nanosec (uint32)
                header["stamp"] = {
                    "sec": struct.unpack_from('<i', data, offset)[0],
                    "nanosec": struct.unpack_from('<I', data, offset + 4)[0]
                }
                offset += 8
                # frame_id string
                offset = (offset + 3) & ~3  # Align to 4 bytes
                length = struct.unpack_from('<I', data, offset)[0]
                offset += 4
                header["frame_id"] = data[offset:offset + length].decode('utf-8', errors='replace')
                offset += length
                value = header
            else:
                # Try to find this type - first in current IDL, then in cache
                decoded_nested = False

                # First try to find the type in the struct map we built
                nested_fields = None
                if struct_map:
                    # Check if any struct name matches the end of our field type
                    # e.g., field_type="some_interfaces_msg_DeviceState" matches struct "DeviceState"
                    for struct_name, struct_fields in struct_map.items():
                        if field_type.endswith(struct_name) or field_type == struct_name:
                            nested_fields = struct_fields
                            break

                if nested_fields:
                        # Found in current IDL
                        try:
                            # Don't align if we don't have enough data
                            if offset + 4 <= len(data):
                                offset = (offset + 3) & ~3
                            remaining_data = data[offset:]
                            nested_result = {}
                            nested_offset = 0

                            for nfield_type, nfield_name in nested_fields:
                                if nested_offset >= len(remaining_data):
                                    break
                                try:
                                    if nfield_type in CDR_TYPE_DECODERS:
                                        decoder = CDR_TYPE_DECODERS[nfield_type]
                                        nested_offset = (nested_offset + decoder["align"] - 1) & ~(decoder["align"] - 1)
                                        if nested_offset + decoder["size"] <= len(remaining_data):
                                            nested_result[nfield_name] = struct.unpack_from(decoder["format"], remaining_data, nested_offset)[0]
                                            nested_offset += decoder["size"]
                                    elif nfield_type == "string":
                                        # Handle string in nested struct
                                        nested_offset = (nested_offset + 3) & ~3
                                        if nested_offset + 4 <= len(remaining_data):
                                            str_len = struct.unpack_from('<I', remaining_data, nested_offset)[0]
                                            nested_offset += 4
                                            if nested_offset + str_len <= len(remaining_data):
                                                nested_result[nfield_name] = remaining_data[nested_offset:nested_offset + str_len].decode('utf-8', errors='replace')
                                                nested_offset += str_len
                                    else:
                                        # Could be another nested type - for now mark as unknown
                                        nested_result[nfield_name] = f"<{nfield_type}>"
                                except Exception as e:
                                    nested_result[nfield_name] = f"<error: {str(e)}>"

                            if nested_result:
                                value = nested_result
                                offset += nested_offset
                                decoded_nested = True
                        except:
                            pass

                if not decoded_nested:
                    value = f"<{field_type}>"

            result[field_name] = value
        except Exception as e:
            result[field_name] = f"<error: {str(e)}>"
            break

    return result


def list_topics(mcap_file, show_progress=True):
    """List all topics in the MCAP file with their types and message counts."""
    try:
        topic_info = defaultdict(lambda: {"type": "unknown", "count": 0})

        open_func = make_open_func(mcap_file)
        with open_func(mcap_file, "rb") as f:
            reader = mcap_reader.make_reader(f)

            # Get summary
            summary = reader.get_summary()
            total_messages = None
            if summary and summary.statistics:
                total_messages = summary.statistics.message_count
                print(f"# MCAP file contains {total_messages} total messages", file=sys.stderr)

            # Create iterator with progress bar if available
            message_iterator = reader.iter_messages()
            if show_progress and TQDM_AVAILABLE and total_messages:
                message_iterator = tqdm(message_iterator, total=total_messages,
                                      desc="Scanning topics", unit=" msgs", file=sys.stderr)

            # Count messages per topic
            for schema, channel, message in message_iterator:
                topic_name = channel.topic
                topic_info[topic_name]["count"] += 1
                if schema and topic_info[topic_name]["type"] == "unknown":
                    topic_info[topic_name]["type"] = schema.name

        # Print a blank line after progress bar if it was shown
        if show_progress and TQDM_AVAILABLE and total_messages:
            print("", file=sys.stderr)

        # Print topic information
        print(f"{'Topic':<50} {'Type':<50} {'Count':>10}")
        print("-" * 112)

        total_count = 0
        for topic, info in sorted(topic_info.items()):
            print(f"{topic:<50} {info['type']:<50} {info['count']:>10}")
            total_count += info["count"]

        print("-" * 112)
        print(f"{'Total':<100} {total_count:>10}")

    except FileNotFoundError:
        print(f"Error: File '{mcap_file}' not found.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading MCAP file: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)



def list_idl_definitions(mcap_file, show_progress=True, specific_topics=None):
    """List IDL definitions for all types or specific topics."""
    open_func = make_open_func(mcap_file)
    try:
        with open_func(mcap_file, "rb") as f:
            # Create decoder factory
            decoder_factory = DecoderFactory()
            reader = mcap_reader.make_reader(f, decoder_factories=[decoder_factory])

            # Get summary
            summary = reader.get_summary()
            if summary and summary.statistics:
                total_messages = summary.statistics.message_count
                print(f"# MCAP file contains {total_messages} messages\n", file=sys.stderr)

            # Collect IDL schemas and map topics
            idl_schemas = {}
            schema_to_topics = {}
            topic_to_schema = {}

            # Get schemas from summary
            if summary and hasattr(summary, 'schemas'):
                for schema_id, schema in summary.schemas.items():
                    if schema.encoding == "ros2idl":
                        idl_schemas[schema_id] = schema
                        schema_to_topics[schema_id] = set()

            # Map topics to schemas
            if summary and hasattr(summary, 'channels'):
                for channel_id, channel in summary.channels.items():
                    if channel.topic and channel.schema_id in idl_schemas:
                        schema_to_topics[channel.schema_id].add(channel.topic)
                        topic_to_schema[channel.topic] = channel.schema_id

            # Filter schemas if specific topics requested
            if specific_topics:
                filtered_schemas = {}
                for topic in specific_topics:
                    if topic in topic_to_schema:
                        schema_id = topic_to_schema[topic]
                        if schema_id in idl_schemas:
                            filtered_schemas[schema_id] = idl_schemas[schema_id]
                schemas_to_display = filtered_schemas
            else:
                schemas_to_display = idl_schemas

            # Display the IDL definitions
            if schemas_to_display:
                print("=" * 80)
                for schema_id in sorted(schemas_to_display.keys()):
                    schema = schemas_to_display[schema_id]
                    topics = schema_to_topics.get(schema_id, set())

                    print(f"Message Type: {schema.name}")
                    print(f"Schema ID: {schema_id}")
                    if topics:
                        print(f"Used in topics: {', '.join(sorted(topics))}")
                    print("-" * 80)

                    # Decode and display IDL
                    idl_text = schema.data.decode('utf-8')
                    print(idl_text)
                    print("=" * 80)
                    print()
            else:
                print("No IDL schemas found", end="")
                if specific_topics:
                    print(f" for topics: {', '.join(specific_topics)}", end="")
                print(".")

    except FileNotFoundError:
        print(f"Error: File '{mcap_file}' not found.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading MCAP file: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


def serialize_message(obj):
    """Recursively serialize ROS message to JSON-compatible dictionary."""
    if obj is None:
        return None
    elif isinstance(obj, (bool, int, float, str)):
        # Primitive types
        return obj
    elif isinstance(obj, bytes):
        # Convert bytes to base64 string for JSON compatibility
        return base64.b64encode(obj).decode('utf-8')
    elif isinstance(obj, (list, tuple)):
        # Handle arrays/lists
        return [serialize_message(item) for item in obj]
    elif hasattr(obj, '__slots__'):
        # Handle ROS messages with slots
        result = {}
        for slot in obj.__slots__:
            value = getattr(obj, slot)
            result[slot] = serialize_message(value)
        return result
    elif hasattr(obj, '__dict__'):
        # Handle objects with __dict__
        result = {}
        for key, value in obj.__dict__.items():
            result[key] = serialize_message(value)
        return result
    elif hasattr(obj, '_fields'):
        # Handle namedtuples
        result = {}
        for field in obj._fields:
            result[field] = serialize_message(getattr(obj, field))
        return result
    else:
        # Fallback: try to convert to string
        return str(obj)


def json_clean_nan(obj):
    """Recursively replace NaN with None"""
    if isinstance(obj, float) and math.isnan(obj):
        return None
    elif isinstance(obj, dict):
        return {k: json_clean_nan(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [json_clean_nan(item) for item in obj]
    return obj


def convert_mcap_to_json(mcap_file, output_file=None, show_progress=True, topics=None, pretty=False, limit=None):
    """Read MCAP file and output each message as JSON object per line.

    Args:
        mcap_file: Path to MCAP file
        output_file: Optional output file path
        show_progress: Whether to show progress bar
        topics: Optional list of topics to include (None means include all)
        pretty: Whether to pretty-print JSON output
        limit: Optional limit on number of JSON objects to output
    """
    try:
        message_count = 0
        decoded_count = 0
        raw_count = 0
        output_count = 0

        # Open output file if specified, otherwise use stdout
        if output_file:
            if output_file.endswith('.bz2'):
                # Use bzip2 compression with highest compression level (9)
                output = bz2.open(output_file, 'wt', compresslevel=9)
            else:
                output = open(output_file, 'w')
        else:
            output = sys.stdout

        open_func = make_open_func(mcap_file)
        try:
            with open_func(mcap_file, "rb") as f:
                # Create decoder factory
                decoder_factory = DecoderFactory()

                # Create reader with decoder factory
                reader = mcap_reader.make_reader(f, decoder_factories=[decoder_factory])

                # Get summary
                summary = reader.get_summary()
                total_messages = None
                if summary and summary.statistics:
                    total_messages = summary.statistics.message_count
                    print(f"# MCAP file contains {total_messages} messages", file=sys.stderr)

                # Decoder cache for performance
                decoder_cache = {}
                # IDL schema cache for custom messages
                idl_cache = {}

                # Pre-populate IDL cache with all available schemas
                if summary and hasattr(summary, 'schemas'):
                    for schema_id, schema in summary.schemas.items():
                        if schema.encoding == "ros2idl":
                            idl_cache[schema_id] = schema

                # Create iterator with progress bar if available and requested
                message_iterator = reader.iter_messages()
                if show_progress and TQDM_AVAILABLE and total_messages:
                    # Adjust progress bar total if limit is specified
                    progress_total = min(total_messages, limit) if limit else total_messages
                    message_iterator = tqdm(message_iterator, total=progress_total,
                                          desc="Converting", unit=" msgs", file=sys.stderr)

                # Iterate through messages
                for schema, channel, message in message_iterator:
                    # Skip if topic filtering is enabled and this topic is not in the list
                    if topics and channel.topic not in topics:
                        continue

                    message_count += 1

                    # Create base JSON object
                    json_obj = {
                        "topic": channel.topic,
                        "timestamp": message.log_time,
                        "message_type": schema.name if schema else "unknown"
                    }

                    # Try to decode the message
                    decoded = False
                    if schema and channel.message_encoding == "cdr":
                        # Check decoder cache first
                        cache_key = (channel.message_encoding, schema.id)
                        decoder = decoder_cache.get(cache_key)

                        if decoder is None:
                            # Try to create decoder for this schema
                            try:
                                decoder = decoder_factory.decoder_for(channel.message_encoding, schema)
                                if decoder:
                                    decoder_cache[cache_key] = decoder
                            except Exception as e:
                                # Mark as failed in cache to avoid retrying
                                decoder_cache[cache_key] = False
                                # Try IDL-based decoding for custom messages
                                if schema.encoding == "ros2idl":
                                    idl_cache[schema.id] = schema

                        # Use decoder if available
                        if decoder and decoder is not False:
                            try:
                                decoded_msg = decoder(message.data)
                                if decoded_msg is not None:
                                    json_obj["data"] = serialize_message(decoded_msg)
                                    decoded = True
                                    decoded_count += 1
                            except Exception as e:
                                json_obj["decode_error"] = f"Decoding failed: {str(e)}"

                    if not decoded:
                        # Try IDL-based decoding if available
                        if schema and schema.encoding == "ros2idl":
                            try:
                                # Always cache the schema
                                if schema.id not in idl_cache:
                                    idl_cache[schema.id] = schema

                                idl_text = schema.data.decode('utf-8')
                                fields = parse_idl_type(idl_text, schema.name)
                                if fields:
                                    decoded_data = decode_cdr_message(message.data, fields, idl_text, idl_cache)
                                    if decoded_data:
                                        json_obj["data"] = decoded_data
                                        decoded = True
                                        decoded_count += 1
                                else:
                                    # No fields found, maybe parsing issue
                                    json_obj["idl_parse_info"] = f"No fields found for {schema.name}"
                                    # Add first 200 chars of IDL for debugging
                                    json_obj["idl_preview"] = idl_text[:200] + "..." if len(idl_text) > 200 else idl_text
                            except Exception as e:
                                json_obj["idl_decode_error"] = str(e)

                        if not decoded:
                            # Output raw data if decoding failed or not attempted
                            raw_count += 1
                            json_obj["data"] = {"raw_data": base64.b64encode(message.data).decode('utf-8')}
                            json_obj["encoding"] = channel.message_encoding
                            if schema:
                                json_obj["schema_encoding"] = schema.encoding

                    # Output JSON (pretty-printed or single line)
                    print(json.dumps(json_clean_nan(json_obj), indent=2 if pretty else None), file=output)
                    output_count += 1

                    # Check if we've reached the limit
                    if limit and output_count >= limit:
                        break

            # Print summary
            summary_msg = f"# Processed {message_count} messages: {decoded_count} decoded, {raw_count} raw"
            if limit and output_count < message_count:
                summary_msg += f" (output limited to {output_count} messages)"
            print(summary_msg, file=sys.stderr)

        finally:
            # Close output file if it was opened
            if output_file and output != sys.stdout:
                output.close()

    except FileNotFoundError:
        print(f"Error: File '{mcap_file}' not found.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading MCAP file: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


def get_basename(path):
    """
    Gets the basename for the path even when the path has an additional .bz2
    file extension, like "mushroom.mcap.bz2".

    Args:
        path (str): The path to the file.
    Returns:
        basename (str): The basename of the path.
    """
    if path.endswith('.bz2'):
        path = path[:-4]
    basename = os.path.splitext(path)[0]
    return basename


def make_open_func(path):
    """
    Returns the appropriate open function based on the file extension.

    Args:
        path (str): The path to the file.
    Returns:
        function: The open function to use (bz2.open or open).
    """
    if path.endswith('.bz2'):
        return bz2.open
    else:
        return open


def process_directory(directory_path, args):
    """Process all MCAP files in a directory recursively.

    Args:
        directory_path: Path to directory containing MCAP files
        args: Command line arguments
    """
    # Find all MCAP files in the directory recursively, including bzip2
    # compressed files.
    mcap_files = glob.glob(os.path.join(directory_path, '**', '*.mcap'), recursive=True) + \
                 glob.glob(os.path.join(directory_path, '**', '*.mcap.bz2'), recursive=True)

    if not mcap_files:
        print(f"Error: No MCAP files found in directory '{directory_path}' (searched recursively)", file=sys.stderr)
        sys.exit(1)

    print(f"# Found {len(mcap_files)} MCAP files in directory (recursive search)", file=sys.stderr)

    # Process each MCAP file
    for i, mcap_file in enumerate(sorted(mcap_files), 1):
        # Generate output filename: basename.json.bz2 in current directory
        basename = get_basename(mcap_file)
        output_file = basename + '.json.bz2'

        # Show relative path for better context in recursive mode
        rel_path = os.path.relpath(mcap_file, directory_path)

        print(f"\n# [{i}/{len(mcap_files)}] Processing: {rel_path} -> {output_file}", file=sys.stderr)

        # Convert topics list to set for faster lookup if provided
        topics_set = set(args.topics_filter) if args.topics_filter else None

        # Convert the file
        convert_mcap_to_json(mcap_file, output_file, show_progress=not args.no_progress,
                            topics=topics_set, pretty=args.pretty, limit=args.limit)

    print(f"\n# Completed processing {len(mcap_files)} files", file=sys.stderr)


def main():

    # Process command line settings
    parser = argparse.ArgumentParser(description="Convert ROS2 MCAP rosbag to JSON format (one object per line)")
    parser.add_argument("-m", "--mcap", required=True, help="Path to the MCAP file or directory containing MCAP files to convert")
    parser.add_argument("-o", "--output", dest="json_file", help="Path to output JSON file (defaults to stdout if not specified). If filename ends with .bz2, output will be compressed with bzip2. When processing a directory, this option is ignored and files are saved as basename.json.bz2")
    parser.add_argument("-q", "--no-progress", action="store_true", help="Disable progress bar (quiet mode)")
    parser.add_argument("-p", "--pretty", action="store_true", help="Pretty-print JSON output (indented format)")
    parser.add_argument("-t", "--topics", action="store_true", help="List all topics with their types and message counts, then exit")
    parser.add_argument("-i", "--idl", action="store_true", help="List IDL definitions of all types contained (or specific topics with subdependencies if provided)")
    parser.add_argument("-l", "--limit", type=int, help="Limit output to N JSON objects")
    parser.add_argument("topics_filter", nargs="*", help="Topics to include in output (if not specified, all topics are included)")
    args = parser.parse_args()

    # Show warning if tqdm is not available but progress bar was requested
    if not args.no_progress and not TQDM_AVAILABLE:
        print("# Warning: tqdm not installed. Install with 'pip install tqdm' for progress bar.", file=sys.stderr)

    # Check if input is a directory
    if os.path.isdir(args.mcap):
        # Directory mode - process all MCAP files
        process_directory(args.mcap, args)

    else:
        # Single file mode - existing behavior
        # If topics listing is requested, list topics and exit
        if args.topics:
            list_topics(args.mcap, show_progress=not args.no_progress)
            sys.exit(0)

        # If IDL definitions are requested, list them and exit
        if args.idl:
            list_idl_definitions(args.mcap, show_progress=not args.no_progress, specific_topics=args.topics_filter)
            sys.exit(0)

        # Convert topics list to set for faster lookup if provided
        topics_set = set(args.topics_filter) if args.topics_filter else None

        # Convert all log entries to JSON
        convert_mcap_to_json(args.mcap, args.json_file, show_progress=not args.no_progress, topics=topics_set, pretty=args.pretty, limit=args.limit)

if __name__ == "__main__":
    main()