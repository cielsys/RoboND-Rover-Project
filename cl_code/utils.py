import numpy as np
import pprint

pp = pprint.PrettyPrinter(indent=4)

# Define a function to convert telemetry strings to float independent of decimal convention
def convert_to_float(string_to_convert):
    if ',' in string_to_convert:
        float_value = np.float(string_to_convert.replace(',', '.'))
    else:
        float_value = np.float(string_to_convert)
    return float_value

def Clip(x, lo, hi):
    return max(lo, min(hi, x))

def Log(strOut):
    pp.pprint(strOut)