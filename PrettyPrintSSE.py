import re

class Vuint8x16Printer(object):
    "Print a Vuint8x16"

    def __init__(self, val):
        self.val = val

    def to_string(self):
        full_string = ""
        for i in range(2):
            word = self.val['values'][i]
            for j in range(8):
                byte = (word >> (j*8)) & 0xFF
                full_string = full_string + " " + str(byte)
        return full_string

    def display_hint(self):
        return 'Vuint8x16'


def sse_lookup_function(val):
    lookup_tag = val.type.tag
    if lookup_tag == None:
        return None
    regex = re.compile("^Vuint8x16$")
    if regex.match(lookup_tag):
        return Vuint8x16Printer(val)
    regex = re.compile("^Vuint16x8$")
    if regex.match(lookup_tag):
        return Vuint8x16Printer(val)
    return None

gdb.pretty_printers.append(sse_lookup_function)