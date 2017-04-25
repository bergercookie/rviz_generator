#!/usr/bin/env python

import sys
import os
import rospy
from misc.custom_exceptions import CustomError, CustomErrorMsg
from displays.RvizDisplay import RvizDisplay
from displays.RvizBundle import RvizBundle
from exceptions import ValueError
from subprocess import call


class RvizGenerator(object):
    """Generate an rviz file based on a given template file.

    Current class sets the generic methods that each RvizGenerator derived
    class should implement.

    """
    def __init__(self, *args, **kargs):
        super(RvizGenerator, self).__init__()
        self.args = args
        self.kargs = kargs

        self._input_displays_at_index = None
        # indentation of the next display to be added
        self._indent_at = 4

        # bare contents of the output rviz file
        self.bare_contents_path_default = \
            os.path.join(os.path.dirname(__file__), "../misc/bare.rviz.template")
        self.bare_contents_path = self.bare_contents_path_default

        # list of displays that have already been added
        self.displays = []
        self.bundles = []

    #
    # bare_contents_path
    #
    @property
    def bare_contents_path(self):
        return self._bare_contents_path

    @bare_contents_path.setter
    def bare_contents_path(self, path):
        if not os.path.isfile(path):
            raise CustomErrorMsg("File {} not found.".format(path))

        # find the position in the file to input the bundles and displays. That
        # should be right after the following *string sequence*:
        seq = ["Visualization Manager:",
               "Class: \"\"",
               "Displays:"]

        with open(path, "r") as f:
            # if the sequence is not present as is, raise an error.
            bare_conts = [" ".join(i.rsplit()) for i in f.readlines()]
            try:
                # strings of seq should be in consecutive lines
                first_index = None
                next_index = None
                for seq_str in seq:
                    index = bare_conts.index(seq_str)
                    if not first_index:
                        first_index = index
                        next_index = first_index
                    else:
                        if index != next_index:
                            raise CustomErrorMsg(
                                "Strings of sequence are not in consecutive lines.")
                    next_index = next_index + 1
            except ValueError:
                raise CustomErrorMsg(
                    "Needed sequence:\n{}\n is not found in the bare contents file. "\
                    .format("\n".join(seq)))

            # index at which to dump the contents for the displays, bundles
            self._input_displays_at_index = next_index

        self._bare_contents_path = path
        self._load_bare_contents()

    #
    # bare_contents
    #
    @property
    def bare_contents(self):
        return self._bare_contents

    @bare_contents.setter
    def bare_contents(self, conts):
        self._bare_contents = conts

    def _load_bare_contents(self):
        with open(self._bare_contents_path) as f:
            self.bare_contents = f.readlines()

    def export_rviz_file(self, pth):
        """Copy the generated rviz file to the filepath indicated.

        """
        # displays
        with open(pth, mode="w") as f:
            # split the bare contents in two - prior and after the displays
            f.writelines(self.bare_contents[:self._input_displays_at_index])

            # Mind the indentation
            for display in self.displays:
                f.writelines([" "*self._indent_at + l for l in display.get_contents()])

            # bundles of displays
            for bundle in self.bundles:
                f.writelines(bundle.get_contents())


            # write the rest of necessary lines
            f.writelines(self.bare_contents[self._input_displays_at_index:])


    def add_display(self, display):
        """Add a new display in the current RvizGenerator instance."""

        if not isinstance(display, RvizDisplay):
            raise CustomErrorMsg(
                "Given instance \"{}\" is not of RvizDisplay type".format(display))
        rospy.loginfo("Adding new display: \"%s\"...", display)
        self.displays.append(display)

    def add_bundle(self, bundle):
        if not isinstance(bundle, RvizBundle):
            raise CustomErrorMsg(
                "Given instance \"{}\" is not of RvizBundle type".format(bundle))

        # TODO

    def launch_rviz(self):
        """
        Launch an rviz instance based on the current contents of the RvizGenerator.

        See also:func:`~RvizGenerator.export_rviz_file`
        """

        pth = "/tmp/generated_file.rviz"
        self.export_rviz_file(pth)
        rospy.logwarn("Launching rviz instance with file: \"%s\"", pth)

        ret = call(["rosrun", "rviz", "rviz", "-d", pth])
        rospy.logwarn("rviz exited with code \"%s\"", ret)



