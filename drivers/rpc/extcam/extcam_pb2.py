# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: extcam.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='extcam.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x0c\x65xtcam.proto\"\x07\n\x05\x45mpty\"G\n\x06\x43\x61mImg\x12\r\n\x05width\x18\x01 \x01(\x05\x12\x0e\n\x06height\x18\x02 \x01(\x05\x12\x0f\n\x07\x63hannel\x18\x03 \x01(\x05\x12\r\n\x05image\x18\x04 \x01(\x0c\x32\"\n\x03\x43\x61m\x12\x1b\n\x06getimg\x12\x06.Empty\x1a\x07.CamImg\"\x00\x62\x06proto3')
)




_EMPTY = _descriptor.Descriptor(
  name='Empty',
  full_name='Empty',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
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
  serialized_start=16,
  serialized_end=23,
)


_CAMIMG = _descriptor.Descriptor(
  name='CamImg',
  full_name='CamImg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='width', full_name='CamImg.width', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='CamImg.height', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='channel', full_name='CamImg.channel', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='image', full_name='CamImg.image', index=3,
      number=4, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=25,
  serialized_end=96,
)

DESCRIPTOR.message_types_by_name['Empty'] = _EMPTY
DESCRIPTOR.message_types_by_name['CamImg'] = _CAMIMG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Empty = _reflection.GeneratedProtocolMessageType('Empty', (_message.Message,), {
  'DESCRIPTOR' : _EMPTY,
  '__module__' : 'extcam_pb2'
  # @@protoc_insertion_point(class_scope:Empty)
  })
_sym_db.RegisterMessage(Empty)

CamImg = _reflection.GeneratedProtocolMessageType('CamImg', (_message.Message,), {
  'DESCRIPTOR' : _CAMIMG,
  '__module__' : 'extcam_pb2'
  # @@protoc_insertion_point(class_scope:CamImg)
  })
_sym_db.RegisterMessage(CamImg)



_CAM = _descriptor.ServiceDescriptor(
  name='Cam',
  full_name='Cam',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=98,
  serialized_end=132,
  methods=[
  _descriptor.MethodDescriptor(
    name='getimg',
    full_name='Cam.getimg',
    index=0,
    containing_service=None,
    input_type=_EMPTY,
    output_type=_CAMIMG,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_CAM)

DESCRIPTOR.services_by_name['Cam'] = _CAM

# @@protoc_insertion_point(module_scope)
