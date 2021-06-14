# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: xarm_shuidi.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='xarm_shuidi.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\nxarm_shuidi.proto\"\x07\n\x05\x45mpty\"P\n\x06Status\x12\"\n\x05value\x18\x01 \x01(\x0e\x32\x13.Status.StatusValue\"\"\n\x0bStatusValue\x12\t\n\x05\x45RROR\x10\x00\x12\x08\n\x04\x44ONE\x10\x01\"\x19\n\tJntValues\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\"3\n\x04Path\x12\x0e\n\x06length\x18\x01 \x01(\x05\x12\r\n\x05njnts\x18\x02 \x01(\x05\x12\x0c\n\x04\x64\x61ta\x18\x03 \x01(\x0c\"0\n\rGripperStatus\x12\r\n\x05speed\x18\x01 \x01(\x05\x12\x10\n\x08position\x18\x02 \x01(\x05\x32\xa9\x01\n\x04XArm\x12$\n\x10move_jspace_path\x12\x05.Path\x1a\x07.Status\"\x00\x12&\n\x0eget_jnt_values\x12\x06.Empty\x1a\n.JntValues\"\x00\x12#\n\x06jaw_to\x12\x0e.GripperStatus\x1a\x07.Status\"\x00\x12.\n\x12get_gripper_status\x12\x06.Empty\x1a\x0e.GripperStatus\"\x00\x62\x06proto3'
)



_STATUS_STATUSVALUE = _descriptor.EnumDescriptor(
  name='StatusValue',
  full_name='Status.StatusValue',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DONE', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=69,
  serialized_end=103,
)
_sym_db.RegisterEnumDescriptor(_STATUS_STATUSVALUE)


_EMPTY = _descriptor.Descriptor(
  name='Empty',
  full_name='Empty',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=14,
  serialized_end=21,
)


_STATUS = _descriptor.Descriptor(
  name='Status',
  full_name='Status',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='value', full_name='Status.value', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _STATUS_STATUSVALUE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=23,
  serialized_end=103,
)


_JNTVALUES = _descriptor.Descriptor(
  name='JntValues',
  full_name='JntValues',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='JntValues.data', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=105,
  serialized_end=130,
)


_PATH = _descriptor.Descriptor(
  name='Path',
  full_name='Path',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='length', full_name='Path.length', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='njnts', full_name='Path.njnts', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='Path.data', index=2,
      number=3, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=132,
  serialized_end=183,
)


_GRIPPERSTATUS = _descriptor.Descriptor(
  name='GripperStatus',
  full_name='GripperStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed', full_name='GripperStatus.speed', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='position', full_name='GripperStatus.position', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=185,
  serialized_end=233,
)

_STATUS.fields_by_name['value'].enum_type = _STATUS_STATUSVALUE
_STATUS_STATUSVALUE.containing_type = _STATUS
DESCRIPTOR.message_types_by_name['Empty'] = _EMPTY
DESCRIPTOR.message_types_by_name['Status'] = _STATUS
DESCRIPTOR.message_types_by_name['JntValues'] = _JNTVALUES
DESCRIPTOR.message_types_by_name['Path'] = _PATH
DESCRIPTOR.message_types_by_name['GripperStatus'] = _GRIPPERSTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Empty = _reflection.GeneratedProtocolMessageType('Empty', (_message.Message,), {
  'DESCRIPTOR' : _EMPTY,
  '__module__' : 'xarm_pb2'
  # @@protoc_insertion_point(class_scope:Empty)
  })
_sym_db.RegisterMessage(Empty)

Status = _reflection.GeneratedProtocolMessageType('Status', (_message.Message,), {
  'DESCRIPTOR' : _STATUS,
  '__module__' : 'xarm_pb2'
  # @@protoc_insertion_point(class_scope:Status)
  })
_sym_db.RegisterMessage(Status)

JntValues = _reflection.GeneratedProtocolMessageType('JntValues', (_message.Message,), {
  'DESCRIPTOR' : _JNTVALUES,
  '__module__' : 'xarm_pb2'
  # @@protoc_insertion_point(class_scope:JntValues)
  })
_sym_db.RegisterMessage(JntValues)

Path = _reflection.GeneratedProtocolMessageType('Path', (_message.Message,), {
  'DESCRIPTOR' : _PATH,
  '__module__' : 'xarm_pb2'
  # @@protoc_insertion_point(class_scope:Path)
  })
_sym_db.RegisterMessage(Path)

GripperStatus = _reflection.GeneratedProtocolMessageType('GripperStatus', (_message.Message,), {
  'DESCRIPTOR' : _GRIPPERSTATUS,
  '__module__' : 'xarm_pb2'
  # @@protoc_insertion_point(class_scope:GripperStatus)
  })
_sym_db.RegisterMessage(GripperStatus)



_XARM = _descriptor.ServiceDescriptor(
  name='XArm',
  full_name='XArm',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=236,
  serialized_end=405,
  methods=[
  _descriptor.MethodDescriptor(
    name='move_jspace_path',
    full_name='XArm.move_jspace_path',
    index=0,
    containing_service=None,
    input_type=_PATH,
    output_type=_STATUS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='get_jnt_values',
    full_name='XArm.get_jnt_values',
    index=1,
    containing_service=None,
    input_type=_EMPTY,
    output_type=_JNTVALUES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='jaw_to',
    full_name='XArm.jaw_to',
    index=2,
    containing_service=None,
    input_type=_GRIPPERSTATUS,
    output_type=_STATUS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='get_gripper_status',
    full_name='XArm.get_gripper_status',
    index=3,
    containing_service=None,
    input_type=_EMPTY,
    output_type=_GRIPPERSTATUS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_XARM)

DESCRIPTOR.services_by_name['XArm'] = _XARM

# @@protoc_insertion_point(module_scope)
