import os
import abc
import sys
MODULES_DIR = os.path.join(os.path.dirname(__file__), "../")
if MODULES_DIR not in sys.path:
    sys.path.insert(0, MODULES_DIR)
from misc.custom_exceptions import CustomErrorMsg


class RvizDisplay(object):
    """Generic class that rivz displays are generated by."""

    # __metaclass__ = abc.ABCMeta

    def __init__(self, rviz_display_name, **kargs):
        """
        Constructor method for RvizDisplay

        :param dict kargs: search and replacement strings meant for modifying
        the template file
        """
        super(RvizDisplay, self).__init__()

        self.prefix = None # img / laser/ odom
        self.rviz_display_name = rviz_display_name

        # template text for each RvizDisplay should be located in
        # the misc/displays/ directory
        self.templ_contents_path = os.path.join(os.path.dirname(__file__),
                                            "../../misc/displays/{}.template"\
                                            .format(rviz_display_name))

        # Strings that are to be found and replaced
        # Note that case in these strings matters. Strings to search for and
        # replace should be uppercase
        self.replace_strings_dict = {
            "DISPLAY_NAME": None,
            "DISPLAY_TOPIC": None
        }
        print "kargs: ", kargs
        self.replace_strings_dict.update(kargs.iteritems())
        print self.replace_strings_dict

    def __str__(self):
        return "Generic RvizDisplay object"


    # TODO - Implement a __copy__ and __deepcopy__ method


    #
    # templ_contents_path
    #
    @property
    def templ_contents_path(self):
        return self._templ_contents_path

    @templ_contents_path.setter
    def templ_contents_path(self, path):
        """
        Manually set the path to the template text for the current
        Rviz Display.

        Method also loads the contents of the given template file.
        """
        if not os.path.isfile(path):
            raise CustomErrorMsg("File {} not found.".format(path))

        self._templ_contents_path = path
        self._load_templ_contents()
        
    #
    # templ_contents
    #
    @property
    def templ_contents(self):
        return self._templ_contents

    @templ_contents.setter
    def templ_contents(self, text):
        self._templ_contents = text


    def _load_templ_contents(self):
        """Load the template text for the current display.

        Current method just loads (not modify) the template contents
        """
        with open(self.templ_contents_path) as f:
            self.templ_contents = f.readlines()


    def modify(self):
        self._modify_templ_contents()


    def _modify_templ_contents(self):
        """
        Modify the given template file according to the variables of the
        corresponding RvizDisplay instance.

        Derived classes probably won't have to override this method in order to
        modify the given file. They just have to add to the
        `replace_strings_dict` whichever string they want to be replaced (with
        the appropriate replacement

        """

        for search_str, rep_str in self.replace_strings_dict.iteritems():
            for i in range(0, len(self.templ_contents)):
                if rep_str is not None:
                    self.templ_contents[i] = \
                        self.templ_contents[i].replace(search_str, rep_str)

        print self.templ_contents


    def get_contents(self, split_lines=True):
        """Return the lines that the current RvizDisplay contains."""

        conts_out = self.templ_contents \
            if split_lines else "\n".join(self.templ_contents)
        return conts_out
