// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice

#include "feetech_ros2/srv/detail/set_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_feetech_ros2
const rosidl_type_hash_t *
feetech_ros2__srv__SetMode__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0xeb, 0x0a, 0xe7, 0xb2, 0x05, 0x9c, 0xb5,
      0x61, 0x76, 0x22, 0x7a, 0x31, 0xd6, 0xce, 0x28,
      0xf4, 0xc4, 0xe6, 0x10, 0x0f, 0x2b, 0x63, 0xdf,
      0x2c, 0xb9, 0x48, 0xf2, 0x8b, 0xa3, 0x79, 0x84,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_feetech_ros2
const rosidl_type_hash_t *
feetech_ros2__srv__SetMode_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x47, 0xf6, 0xc4, 0xe5, 0x78, 0xb5, 0x9d, 0x5c,
      0xd9, 0x21, 0x52, 0x00, 0x65, 0xdf, 0xc1, 0x11,
      0x50, 0x6c, 0xfe, 0x35, 0x26, 0x16, 0xed, 0xc5,
      0x49, 0x87, 0x65, 0x39, 0x2f, 0xbc, 0xe7, 0x92,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_feetech_ros2
const rosidl_type_hash_t *
feetech_ros2__srv__SetMode_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc1, 0xdd, 0x62, 0xe3, 0xf6, 0xc8, 0xf1, 0xf7,
      0x84, 0xf5, 0x1b, 0x9a, 0xc9, 0xb3, 0xcc, 0x6c,
      0xaa, 0x53, 0x4c, 0xc3, 0x3e, 0x67, 0x3a, 0x89,
      0x18, 0x53, 0x8b, 0x25, 0x29, 0x6c, 0x06, 0xff,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_feetech_ros2
const rosidl_type_hash_t *
feetech_ros2__srv__SetMode_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x44, 0x47, 0x28, 0xea, 0xc8, 0x3e, 0xd3, 0x3a,
      0xfd, 0xd3, 0x4b, 0xe9, 0xea, 0x57, 0x2a, 0x4a,
      0x5f, 0x5c, 0x57, 0xb4, 0xcb, 0x02, 0xd4, 0x19,
      0x27, 0x4d, 0xd8, 0x50, 0xa0, 0xb2, 0xe0, 0x4f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char feetech_ros2__srv__SetMode__TYPE_NAME[] = "feetech_ros2/srv/SetMode";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char feetech_ros2__srv__SetMode_Event__TYPE_NAME[] = "feetech_ros2/srv/SetMode_Event";
static char feetech_ros2__srv__SetMode_Request__TYPE_NAME[] = "feetech_ros2/srv/SetMode_Request";
static char feetech_ros2__srv__SetMode_Response__TYPE_NAME[] = "feetech_ros2/srv/SetMode_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char feetech_ros2__srv__SetMode__FIELD_NAME__request_message[] = "request_message";
static char feetech_ros2__srv__SetMode__FIELD_NAME__response_message[] = "response_message";
static char feetech_ros2__srv__SetMode__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field feetech_ros2__srv__SetMode__FIELDS[] = {
  {
    {feetech_ros2__srv__SetMode__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {feetech_ros2__srv__SetMode_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription feetech_ros2__srv__SetMode__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
feetech_ros2__srv__SetMode__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {feetech_ros2__srv__SetMode__TYPE_NAME, 24, 24},
      {feetech_ros2__srv__SetMode__FIELDS, 3, 3},
    },
    {feetech_ros2__srv__SetMode__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = feetech_ros2__srv__SetMode_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = feetech_ros2__srv__SetMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = feetech_ros2__srv__SetMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char feetech_ros2__srv__SetMode_Request__FIELD_NAME__operating_mode[] = "operating_mode";

static rosidl_runtime_c__type_description__Field feetech_ros2__srv__SetMode_Request__FIELDS[] = {
  {
    {feetech_ros2__srv__SetMode_Request__FIELD_NAME__operating_mode, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
feetech_ros2__srv__SetMode_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
      {feetech_ros2__srv__SetMode_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char feetech_ros2__srv__SetMode_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field feetech_ros2__srv__SetMode_Response__FIELDS[] = {
  {
    {feetech_ros2__srv__SetMode_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
feetech_ros2__srv__SetMode_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
      {feetech_ros2__srv__SetMode_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char feetech_ros2__srv__SetMode_Event__FIELD_NAME__info[] = "info";
static char feetech_ros2__srv__SetMode_Event__FIELD_NAME__request[] = "request";
static char feetech_ros2__srv__SetMode_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field feetech_ros2__srv__SetMode_Event__FIELDS[] = {
  {
    {feetech_ros2__srv__SetMode_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription feetech_ros2__srv__SetMode_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
feetech_ros2__srv__SetMode_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {feetech_ros2__srv__SetMode_Event__TYPE_NAME, 30, 30},
      {feetech_ros2__srv__SetMode_Event__FIELDS, 3, 3},
    },
    {feetech_ros2__srv__SetMode_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = feetech_ros2__srv__SetMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = feetech_ros2__srv__SetMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int64 operating_mode\n"
  "---\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
feetech_ros2__srv__SetMode__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {feetech_ros2__srv__SetMode__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 37, 37},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
feetech_ros2__srv__SetMode_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {feetech_ros2__srv__SetMode_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
feetech_ros2__srv__SetMode_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {feetech_ros2__srv__SetMode_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
feetech_ros2__srv__SetMode_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {feetech_ros2__srv__SetMode_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
feetech_ros2__srv__SetMode__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *feetech_ros2__srv__SetMode__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *feetech_ros2__srv__SetMode_Event__get_individual_type_description_source(NULL);
    sources[3] = *feetech_ros2__srv__SetMode_Request__get_individual_type_description_source(NULL);
    sources[4] = *feetech_ros2__srv__SetMode_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
feetech_ros2__srv__SetMode_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *feetech_ros2__srv__SetMode_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
feetech_ros2__srv__SetMode_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *feetech_ros2__srv__SetMode_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
feetech_ros2__srv__SetMode_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *feetech_ros2__srv__SetMode_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *feetech_ros2__srv__SetMode_Request__get_individual_type_description_source(NULL);
    sources[3] = *feetech_ros2__srv__SetMode_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
