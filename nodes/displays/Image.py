from RvizDisplay import RvizDisplay

class Image(RvizDisplay):
    """Wrapper for the Image display"""
    def __init__(self, **kargs):
        super(Image, self).__init__(rviz_display_name="Image", **kargs)
        self.prefix = "img"


    def __str__(self):
        return "Generic Image object"
