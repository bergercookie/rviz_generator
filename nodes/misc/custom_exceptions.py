class CustomError(BaseException):
    """Class for implementing custom exceptions"""
    pass

class CustomErrorMsg(CustomError):
    """Custom exception with a corresponding message."""

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        error_msg = self.msg
        return error_msg
