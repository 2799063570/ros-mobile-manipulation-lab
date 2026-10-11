"""ROS message to ordinary JSON values, with no extra converter dependency."""
def to_dict(value):
    if hasattr(value, '__slots__'):
        return {slot:to_dict(getattr(value,slot)) for slot in value.__slots__}
    if isinstance(value, (list,tuple,bytes,bytearray)):
        return [to_dict(v) for v in value]
    if isinstance(value, dict):
        return {k:to_dict(v) for k,v in value.items()}
    return value
