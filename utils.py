
import math

# Based on this: https://stackoverflow.com/questions/72843843/how-do-i-make-lcm-works-with-floating-numbers
def lcm_float(a, b):
    """Calculate the LCM of two floats."""

    # Find the GCD of the floats by multiplying by a large power of 10
    # and converting to integers
    multiplier = 10**max(len(str(a).split('.')[1]), len(str(b).split('.')[1]))
    a_int = int(a * multiplier)
    b_int = int(b * multiplier)

    gcd = math.gcd(a_int, b_int)

    # Calculate the LCM using the GCD
    lcm = (a_int * b_int) // gcd

    # Divide the LCM by the multiplier to get the LCM of the original floats
    return lcm / multiplier

# Based on https://www.geeksforgeeks.org/program-find-gcd-floating-point-numbers/
def gcd_float(a, b):
    # Find the GCD of the floats by multiplying by a large power of 10
    # and converting to integers
    multiplier = 10**max(len(str(a).split('.')[1]), len(str(b).split('.')[1]))
    a_int = int(a * multiplier)
    b_int = int(b * multiplier)

    gcd = math.gcd(a_int, b_int)

    # Divide the GCD by the multiplier to get the GCD of the original floats
    return gcd / multiplier
