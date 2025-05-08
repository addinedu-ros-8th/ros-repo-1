// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from nuri_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#include "nuri_msgs/msg/detail/robot_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_nuri_msgs
const rosidl_type_hash_t *
nuri_msgs__msg__RobotState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x25, 0x0c, 0x90, 0x42, 0xee, 0x8a, 0xfe, 0x8f,
      0x5d, 0x45, 0xac, 0xf4, 0x52, 0xd1, 0x91, 0xda,
      0x7f, 0xd2, 0x90, 0x28, 0x97, 0x66, 0x4d, 0x50,
      0x22, 0x97, 0x16, 0xb7, 0x25, 0x83, 0x16, 0x56,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char nuri_msgs__msg__RobotState__TYPE_NAME[] = "nuri_msgs/msg/RobotState";

// Define type names, field names, and default values
static char nuri_msgs__msg__RobotState__FIELD_NAME__status[] = "status";
static char nuri_msgs__msg__RobotState__FIELD_NAME__battery[] = "battery";

static rosidl_runtime_c__type_description__Field nuri_msgs__msg__RobotState__FIELDS[] = {
  {
    {nuri_msgs__msg__RobotState__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nuri_msgs__msg__RobotState__FIELD_NAME__battery, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuri_msgs__msg__RobotState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuri_msgs__msg__RobotState__TYPE_NAME, 24, 24},
      {nuri_msgs__msg__RobotState__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Robot State Msg\n"
  "\n"
  "string status\n"
  "int32 battery";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
nuri_msgs__msg__RobotState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuri_msgs__msg__RobotState__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 46, 46},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuri_msgs__msg__RobotState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuri_msgs__msg__RobotState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
