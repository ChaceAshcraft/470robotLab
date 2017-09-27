from sys import platform


__all__ = ['BTComm']

if platform == 'linux':
    from btcomm.linux_socket import BTComm
elif platform == 'darwin':
    from btcomm.macos_serial import BTComm
else:
    raise ImportError('No bluetooth support for platform {}'.format(platform))
