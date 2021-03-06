# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

import robotconn.rpc.nxtrobot.nxtrobot_pb2 as nxtrobot__pb2


class NxtStub(object):
  # missing associated documentation comment in .proto file
  pass

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.checkEncoders = channel.unary_unary(
        '/Nxt/checkEncoders',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.servoOn = channel.unary_unary(
        '/Nxt/servoOn',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.servoOff = channel.unary_unary(
        '/Nxt/servoOff',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.goInitial = channel.unary_unary(
        '/Nxt/goInitial',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.goOffPose = channel.unary_unary(
        '/Nxt/goOffPose',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.getJointAngles = channel.unary_unary(
        '/Nxt/getJointAngles',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.ReturnValue.FromString,
        )
    self.setJointAngles = channel.unary_unary(
        '/Nxt/setJointAngles',
        request_serializer=nxtrobot__pb2.SendValue.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.playPattern = channel.unary_unary(
        '/Nxt/playPattern',
        request_serializer=nxtrobot__pb2.SendValue.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.closeHandToolLft = channel.unary_unary(
        '/Nxt/closeHandToolLft',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.closeHandToolRgt = channel.unary_unary(
        '/Nxt/closeHandToolRgt',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.openHandToolLft = channel.unary_unary(
        '/Nxt/openHandToolLft',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.openHandToolRgt = channel.unary_unary(
        '/Nxt/openHandToolRgt',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.ejectHandToolLft = channel.unary_unary(
        '/Nxt/ejectHandToolLft',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.ejectHandToolRgt = channel.unary_unary(
        '/Nxt/ejectHandToolRgt',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.attachHandToolLft = channel.unary_unary(
        '/Nxt/attachHandToolLft',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )
    self.attachHandToolRgt = channel.unary_unary(
        '/Nxt/attachHandToolRgt',
        request_serializer=nxtrobot__pb2.Empty.SerializeToString,
        response_deserializer=nxtrobot__pb2.Status.FromString,
        )


class NxtServicer(object):
  # missing associated documentation comment in .proto file
  pass

  def checkEncoders(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def servoOn(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def servoOff(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def goInitial(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def goOffPose(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def getJointAngles(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def setJointAngles(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def playPattern(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def closeHandToolLft(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def closeHandToolRgt(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def openHandToolLft(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def openHandToolRgt(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def ejectHandToolLft(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def ejectHandToolRgt(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def attachHandToolLft(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def attachHandToolRgt(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_NxtServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'checkEncoders': grpc.unary_unary_rpc_method_handler(
          servicer.checkEncoders,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'servoOn': grpc.unary_unary_rpc_method_handler(
          servicer.servoOn,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'servoOff': grpc.unary_unary_rpc_method_handler(
          servicer.servoOff,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'goInitial': grpc.unary_unary_rpc_method_handler(
          servicer.goInitial,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'goOffPose': grpc.unary_unary_rpc_method_handler(
          servicer.goOffPose,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'getJointAngles': grpc.unary_unary_rpc_method_handler(
          servicer.getJointAngles,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.ReturnValue.SerializeToString,
      ),
      'setJointAngles': grpc.unary_unary_rpc_method_handler(
          servicer.setJointAngles,
          request_deserializer=nxtrobot__pb2.SendValue.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'playPattern': grpc.unary_unary_rpc_method_handler(
          servicer.playPattern,
          request_deserializer=nxtrobot__pb2.SendValue.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'closeHandToolLft': grpc.unary_unary_rpc_method_handler(
          servicer.closeHandToolLft,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'closeHandToolRgt': grpc.unary_unary_rpc_method_handler(
          servicer.closeHandToolRgt,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'openHandToolLft': grpc.unary_unary_rpc_method_handler(
          servicer.openHandToolLft,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'openHandToolRgt': grpc.unary_unary_rpc_method_handler(
          servicer.openHandToolRgt,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'ejectHandToolLft': grpc.unary_unary_rpc_method_handler(
          servicer.ejectHandToolLft,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'ejectHandToolRgt': grpc.unary_unary_rpc_method_handler(
          servicer.ejectHandToolRgt,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'attachHandToolLft': grpc.unary_unary_rpc_method_handler(
          servicer.attachHandToolLft,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
      'attachHandToolRgt': grpc.unary_unary_rpc_method_handler(
          servicer.attachHandToolRgt,
          request_deserializer=nxtrobot__pb2.Empty.FromString,
          response_serializer=nxtrobot__pb2.Status.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'Nxt', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
