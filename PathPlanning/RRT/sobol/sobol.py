"""
  Licensing:
    This code is distributed under the MIT license.

  Authors:
    Original FORTRAN77 version of i4_sobol by Bennett Fox.
    MATLAB version by John Burkardt.
    PYTHON version by Corrado Chisari

    Original Python version of is_prime by Corrado Chisari

    Original MATLAB versions of other functions by John Burkardt.
    PYTHON versions by Corrado Chisari

    Original code is available at
    http://people.sc.fsu.edu/~jburkardt/py_src/sobol/sobol.html

    Note: the i4 prefix means that the function takes a numeric argument or
          returns a number which is interpreted inside the function as a 4
          byte integer
    Note: the r4 prefix means that the function takes a numeric argument or
          returns a number which is interpreted inside the function as a 4
          byte float
"""
import math
import sys
import numpy as np

atmost = None
dim_max = None
dim_num_save = None
initialized = None
lastq = None
log_max = None
maxcol = None
poly = None
recipd = None
seed_save = None
v = None


def i4_bit_hi1(n):
    """
     I4_BIT_HI1 returns the position of the high 1 bit base 2 in an I4.

      Discussion:

        An I4 is an integer ( kind = 4 ) value.

      Example:

           N    Binary    Hi 1
        ----    --------  ----
           0           0     0
           1           1     1
           2          10     2
           3          11     2
           4         100     3
           5         101     3
           6         110     3
           7         111     3
           8        1000     4
           9        1001     4
          10        1010     4
          11        1011     4
          12        1100     4
          13        1101     4
          14        1110     4
          15        1111     4
          16       10000     5
          17       10001     5
        1023  1111111111    10
        1024 10000000000    11
        1025 10000000001    11

      Licensing:

        This code is distributed under the GNU LGPL license.

      Modified:

        26 October 2014

      Author:

        John Burkardt

      Parameters:

        Input, integer N, the integer to be measured.
        N should be nonnegative.  If N is nonpositive, the function
        will always be 0.

        Output, integer BIT, the position of the highest bit.

    """
    i = n
    bit = 0

    while True:

        if i <= 0:
            break

        bit = bit + 1
        i = i // 2

    return bit


def i4_bit_lo0(n):
    """
     I4_BIT_LO0 returns the position of the low 0 bit base 2 in an I4.

      Discussion:

        An I4 is an integer ( kind = 4 ) value.

      Example:

           N    Binary    Lo 0
        ----    --------  ----
           0           0     1
           1           1     2
           2          10     1
           3          11     3
           4         100     1
           5         101     2
           6         110     1
           7         111     4
           8        1000     1
           9        1001     2
          10        1010     1
          11        1011     3
          12        1100     1
          13        1101     2
          14        1110     1
          15        1111     5
          16       10000     1
          17       10001     2
        1023  1111111111    11
        1024 10000000000     1
        1025 10000000001     2

      Licensing:

        This code is distributed under the GNU LGPL license.

      Modified:

        08 February 2018

      Author:

        John Burkardt

      Parameters:

        Input, integer N, the integer to be measured.
        N should be nonnegative.

        Output, integer BIT, the position of the low 1 bit.

    """
    bit = 0
    i = n

    while True:

        bit = bit + 1
        i2 = i // 2

        if i == 2 * i2:
            break

        i = i2

    return bit


def i4_sobol_generate(m, n, skip):
    """


     I4_SOBOL_GENERATE generates a Sobol dataset.

      Licensing:

        This code is distributed under the MIT license.

      Modified:

        22 February 2011

      Author:

        Original MATLAB version by John Burkardt.
        PYTHON version by Corrado Chisari

      Parameters:

        Input, integer M, the spatial dimension.

        Input, integer N, the number of points to generate.

        Input, integer SKIP, the number of initial points to skip.

        Output, real R(M,N), the points.

    """
    r = np.zeros((m, n))
    for j in range(1, n + 1):
        seed = skip + j - 2
        [r[0:m, j - 1], seed] = i4_sobol(m, seed)
    return r


def i4_sobol(dim_num, seed):
    """


     I4_SOBOL generates a new quasirandom Sobol vector with each call.

      Discussion:

        The routine adapts the ideas of Antonov and Saleev.

      Licensing:

        This code is distributed under the MIT license.

      Modified:

        22 February 2011

      Author:

        Original FORTRAN77 version by Bennett Fox.
        MATLAB version by John Burkardt.
        PYTHON version by Corrado Chisari

      Reference:

        Antonov, Saleev,
        USSR Computational Mathematics and Mathematical Physics,
        olume 19, 19, pages 252 - 256.

        Paul Bratley, Bennett Fox,
        Algorithm 659:
        Implementing Sobol's Quasirandom Sequence Generator,
        ACM Transactions on Mathematical Software,
        Volume 14, Number 1, pages 88-100, 1988.

        Bennett Fox,
        Algorithm 647:
        Implementation and Relative Efficiency of Quasirandom
        Sequence Generators,
        ACM Transactions on Mathematical Software,
        Volume 12, Number 4, pages 362-376, 1986.

        Ilya Sobol,
        USSR Computational Mathematics and Mathematical Physics,
        Volume 16, pages 236-242, 1977.

        Ilya Sobol, Levitan,
        The Production of Points Uniformly Distributed in a Multidimensional
        Cube (in Russian),
        Preprint IPM Akad. Nauk SSSR,
        Number 40, Moscow 1976.

      Parameters:

        Input, integer DIM_NUM, the number of spatial dimensions.
        DIM_NUM must satisfy 1 <= DIM_NUM <= 40.

        Input/output, integer SEED, the "seed" for the sequence.
        This is essentially the index in the sequence of the quasirandom
        value to be generated.    On output, SEED has been set to the
        appropriate next value, usually simply SEED+1.
        If SEED is less than 0 on input, it is treated as though it were 0.
        An input value of 0 requests the first (0-th) element of the sequence.

        Output, real QUASI(DIM_NUM), the next quasirandom vector.

    """

    global atmost
    global dim_max
    global dim_num_save
    global initialized
    global lastq
    global log_max
    global maxcol
    global poly
    global recipd
    global seed_save
    global v

    if not initialized or dim_num != dim_num_save:
        initialized = 1
        dim_max = 40
        dim_num_save = -1
        log_max = 30
        seed_save = -1
        #
        #    Initialize (part of) V.
        #
        v = np.zeros((dim_max, log_max))
        v[0:40, 0] = np.transpose([
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
        ])

        v[2:40, 1] = np.transpose([
            1, 3, 1, 3, 1, 3, 3, 1, 3, 1, 3, 1, 3, 1, 1, 3, 1, 3, 1, 3, 1, 3,
            3, 1, 3, 1, 3, 1, 3, 1, 1, 3, 1, 3, 1, 3, 1, 3
        ])

        v[3:40, 2] = np.transpose([
            7, 5, 1, 3, 3, 7, 5, 5, 7, 7, 1, 3, 3, 7, 5, 1, 1, 5, 3, 3, 1, 7,
            5, 1, 3, 3, 7, 5, 1, 1, 5, 7, 7, 5, 1, 3, 3
        ])

        v[5:40, 3] = np.transpose([
            1, 7, 9, 13, 11, 1, 3, 7, 9, 5, 13, 13, 11, 3, 15, 5, 3, 15, 7, 9,
            13, 9, 1, 11, 7, 5, 15, 1, 15, 11, 5, 3, 1, 7, 9
        ])

        v[7:40, 4] = np.transpose([
            9, 3, 27, 15, 29, 21, 23, 19, 11, 25, 7, 13, 17, 1, 25, 29, 3, 31,
            11, 5, 23, 27, 19, 21, 5, 1, 17, 13, 7, 15, 9, 31, 9
        ])

        v[13:40, 5] = np.transpose([
            37, 33, 7, 5, 11, 39, 63, 27, 17, 15, 23, 29, 3, 21, 13, 31, 25, 9,
            49, 33, 19, 29, 11, 19, 27, 15, 25
        ])

        v[19:40, 6] = np.transpose([
            13, 33, 115, 41, 79, 17, 29, 119, 75, 73, 105, 7, 59, 65, 21, 3,
            113, 61, 89, 45, 107
        ])

        v[37:40, 7] = np.transpose([7, 23, 39])
        #
        #    Set POLY.
        #
        poly = [
            1, 3, 7, 11, 13, 19, 25, 37, 59, 47, 61, 55, 41, 67, 97, 91, 109,
            103, 115, 131, 193, 137, 145, 143, 241, 157, 185, 167, 229, 171,
            213, 191, 253, 203, 211, 239, 247, 285, 369, 299
        ]

        atmost = 2**log_max - 1
        #
        #    Find the number of bits in ATMOST.
        #
        maxcol = i4_bit_hi1(atmost)
        #
        #    Initialize row 1 of V.
        #
        v[0, 0:maxcol] = 1

        # Things to do only if the dimension changed.

    if dim_num != dim_num_save:
        #
        #    Check parameters.
        #
        if (dim_num < 1 or dim_max < dim_num):
            print('I4_SOBOL - Fatal error!')
            print('    The spatial dimension DIM_NUM should satisfy:')
            print('        1 <= DIM_NUM <= %d' % dim_max)
            print('    But this input value is DIM_NUM = %d' % dim_num)
            return None

        dim_num_save = dim_num
        #
        #    Initialize the remaining rows of V.
        #
        for i in range(2, dim_num + 1):
            #
            #    The bits of the integer POLY(I) gives the form of polynomial
            #    I.
            #
            #    Find the degree of polynomial I from binary encoding.
            #
            j = poly[i - 1]
            m = 0
            while True:
                j = math.floor(j / 2.)
                if (j <= 0):
                    break
                m = m + 1
            #
            #    Expand this bit pattern to separate components of the logical
            #    array INCLUD.
            #
            j = poly[i - 1]
            includ = np.zeros(m)
            for k in range(m, 0, -1):
                j2 = math.floor(j / 2.)
                includ[k - 1] = (j != 2 * j2)
                j = j2
            #
            #    Calculate the remaining elements of row I as explained
            #    in Bratley and Fox, section 2.
            #
            for j in range(m + 1, maxcol + 1):
                newv = v[i - 1, j - m - 1]
                l_var = 1
                for k in range(1, m + 1):
                    l_var = 2 * l_var
                    if (includ[k - 1]):
                        newv = np.bitwise_xor(
                            int(newv), int(l_var * v[i - 1, j - k - 1]))
                v[i - 1, j - 1] = newv
#
#    Multiply columns of V by appropriate power of 2.
#
        l_var = 1
        for j in range(maxcol - 1, 0, -1):
            l_var = 2 * l_var
            v[0:dim_num, j - 1] = v[0:dim_num, j - 1] * l_var
#
#    RECIPD is 1/(common denominator of the elements in V).
#
        recipd = 1.0 / (2 * l_var)
        lastq = np.zeros(dim_num)

    seed = int(math.floor(seed))

    if (seed < 0):
        seed = 0

    if (seed == 0):
        l_var = 1
        lastq = np.zeros(dim_num)

    elif (seed == seed_save + 1):
        #
        #    Find the position of the right-hand zero in SEED.
        #
        l_var = i4_bit_lo0(seed)

    elif (seed <= seed_save):

        seed_save = 0
        lastq = np.zeros(dim_num)

        for seed_temp in range(int(seed_save), int(seed)):
            l_var = i4_bit_lo0(seed_temp)
            for i in range(1, dim_num + 1):
                lastq[i - 1] = np.bitwise_xor(
                    int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

        l_var = i4_bit_lo0(seed)

    elif (seed_save + 1 < seed):

        for seed_temp in range(int(seed_save + 1), int(seed)):
            l_var = i4_bit_lo0(seed_temp)
            for i in range(1, dim_num + 1):
                lastq[i - 1] = np.bitwise_xor(
                    int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

        l_var = i4_bit_lo0(seed)
#
#    Check that the user is not calling too many times!
#
    if maxcol < l_var:
        print('I4_SOBOL - Fatal error!')
        print('    Too many calls!')
        print('    MAXCOL = %d\n' % maxcol)
        print('    L =            %d\n' % l_var)
        return None


#
#    Calculate the new components of QUASI.
#
    quasi = np.zeros(dim_num)
    for i in range(1, dim_num + 1):
        quasi[i - 1] = lastq[i - 1] * recipd
        lastq[i - 1] = np.bitwise_xor(
            int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

    seed_save = seed
    seed = seed + 1

    return [quasi, seed]


def i4_uniform_ab(a, b, seed):
    """


     I4_UNIFORM_AB returns a scaled pseudorandom I4.

      Discussion:

        The pseudorandom number will be scaled to be uniformly distributed
        between A and B.

      Licensing:

        This code is distributed under the GNU LGPL license.

      Modified:

        05 April 2013

      Author:

        John Burkardt

      Reference:

        Paul Bratley, Bennett Fox, Linus Schrage,
        A Guide to Simulation,
        Second Edition,
        Springer, 1987,
        ISBN: 0387964673,
        LC: QA76.9.C65.B73.

        Bennett Fox,
        Algorithm 647:
        Implementation and Relative Efficiency of Quasirandom
        Sequence Generators,
        ACM Transactions on Mathematical Software,
        Volume 12, Number 4, December 1986, pages 362-376.

        Pierre L'Ecuyer,
        Random Number Generation,
        in Handbook of Simulation,
        edited by Jerry Banks,
        Wiley, 1998,
        ISBN: 0471134031,
        LC: T57.62.H37.

        Peter Lewis, Allen Goodman, James Miller,
        A Pseudo-Random Number Generator for the System/360,
        IBM Systems Journal,
        Volume 8, Number 2, 1969, pages 136-143.

      Parameters:

        Input, integer A, B, the minimum and maximum acceptable values.

        Input, integer SEED, a seed for the random number generator.

        Output, integer C, the randomly chosen integer.

        Output, integer SEED, the updated seed.

    """

    i4_huge = 2147483647

    seed = int(seed)

    seed = (seed % i4_huge)

    if seed < 0:
        seed = seed + i4_huge

    if seed == 0:
        print('')
        print('I4_UNIFORM_AB - Fatal error!')
        print('  Input SEED = 0!')
        sys.exit('I4_UNIFORM_AB - Fatal error!')

    k = (seed // 127773)

    seed = 167 * (seed - k * 127773) - k * 2836

    if seed < 0:
        seed = seed + i4_huge

    r = seed * 4.656612875E-10
    #
    #  Scale R to lie between A-0.5 and B+0.5.
    #
    a = round(a)
    b = round(b)

    r = (1.0 - r) * (min(a, b) - 0.5) \
        + r * (max(a, b) + 0.5)
    #
    #  Use rounding to convert R to an integer between A and B.
    #
    value = round(r)

    value = max(value, min(a, b))
    value = min(value, max(a, b))
    value = int(value)

    return value, seed


def prime_ge(n):
    """


     PRIME_GE returns the smallest prime greater than or equal to N.

      Example:

          N    PRIME_GE

        -10     2
          1     2
          2     2
          3     3
          4     5
          5     5
          6     7
          7     7
          8    11
          9    11
         10    11

      Licensing:

        This code is distributed under the MIT license.

      Modified:

        22 February 2011

      Author:

        Original MATLAB version by John Burkardt.
        PYTHON version by Corrado Chisari

      Parameters:

        Input, integer N, the number to be bounded.

        Output, integer P, the smallest prime number that is greater
        than or equal to N.

    """
    p = max(math.ceil(n), 2)
    while not isprime(p):
        p = p + 1

    return p


def isprime(n):
    """


     IS_PRIME returns True if N is a prime number, False otherwise

      Licensing:

        This code is distributed under the MIT license.

      Modified:

        22 February 2011

      Author:

        Corrado Chisari

      Parameters:

        Input, integer N, the number to be checked.

        Output, boolean value, True or False

    """
    if n != int(n) or n < 1:
        return False
    p = 2
    while p < n:
        if n % p == 0:
            return False
        p += 1

    return True


def r4_uniform_01(seed):
    """


     R4_UNIFORM_01 returns a unit pseudorandom R4.

      Discussion:

        This routine implements the recursion

          seed = 167 * seed mod ( 2^31 - 1 )
          r = seed / ( 2^31 - 1 )

        The integer arithmetic never requires more than 32 bits,
        including a sign bit.

        If the initial seed is 12345, then the first three computations are

          Input     Output      R4_UNIFORM_01
          SEED      SEED

             12345   207482415  0.096616
         207482415  1790989824  0.833995
        1790989824  2035175616  0.947702

      Licensing:

        This code is distributed under the GNU LGPL license.

      Modified:

        04 April 2013

      Author:

        John Burkardt

      Reference:

        Paul Bratley, Bennett Fox, Linus Schrage,
        A Guide to Simulation,
        Second Edition,
        Springer, 1987,
        ISBN: 0387964673,
        LC: QA76.9.C65.B73.

        Bennett Fox,
        Algorithm 647:
        Implementation and Relative Efficiency of Quasirandom
        Sequence Generators,
        ACM Transactions on Mathematical Software,
        Volume 12, Number 4, December 1986, pages 362-376.

        Pierre L'Ecuyer,
        Random Number Generation,
        in Handbook of Simulation,
        edited by Jerry Banks,
        Wiley, 1998,
        ISBN: 0471134031,
        LC: T57.62.H37.

        Peter Lewis, Allen Goodman, James Miller,
        A Pseudo-Random Number Generator for the System/360,
        IBM Systems Journal,
        Volume 8, Number 2, 1969, pages 136-143.

      Parameters:

        Input, integer SEED, the integer "seed" used to generate
        the output random number.  SEED should not be 0.

        Output, real R, a random value between 0 and 1.

        Output, integer SEED, the updated seed.  This would
        normally be used as the input seed on the next call.

    """

    i4_huge = 2147483647

    if (seed == 0):
        print('')
        print('R4_UNIFORM_01 - Fatal error!')
        print('  Input SEED = 0!')
        sys.exit('R4_UNIFORM_01 - Fatal error!')

    seed = (seed % i4_huge)

    if seed < 0:
        seed = seed + i4_huge

    k = (seed // 127773)

    seed = 167 * (seed - k * 127773) - k * 2836

    if seed < 0:
        seed = seed + i4_huge

    r = seed * 4.656612875E-10

    return r, seed


def r8mat_write(filename, m, n, a):
    """


     R8MAT_WRITE writes an R8MAT to a file.

      Licensing:

        This code is distributed under the GNU LGPL license.

      Modified:

        12 October 2014

      Author:

        John Burkardt

      Parameters:

        Input, string FILENAME, the name of the output file.

        Input, integer M, the number of rows in A.

        Input, integer N, the number of columns in A.

        Input, real A(M,N), the matrix.
    """

    with open(filename, 'w') as output:
        for i in range(0, m):
            for j in range(0, n):
                s = '  %g' % (a[i, j])
                output.write(s)
            output.write('\n')


def tau_sobol(dim_num):
    """


     TAU_SOBOL defines favorable starting seeds for Sobol sequences.

      Discussion:

        For spatial dimensions 1 through 13, this routine returns
        a "favorable" value TAU by which an appropriate starting point
        in the Sobol sequence can be determined.

        These starting points have the form N = 2**K, where
        for integration problems, it is desirable that
                TAU + DIM_NUM - 1 <= K
        while for optimization problems, it is desirable that
                TAU < K.

      Licensing:

        This code is distributed under the MIT license.

      Modified:

        22 February 2011

      Author:

        Original FORTRAN77 version by Bennett Fox.
        MATLAB version by John Burkardt.
        PYTHON version by Corrado Chisari

      Reference:

        IA Antonov, VM Saleev,
        USSR Computational Mathematics and Mathematical Physics,
        Volume 19, 19, pages 252 - 256.

        Paul Bratley, Bennett Fox,
        Algorithm 659:
        Implementing Sobol's Quasirandom Sequence Generator,
        ACM Transactions on Mathematical Software,
        Volume 14, Number 1, pages 88-100, 1988.

        Bennett Fox,
        Algorithm 647:
        Implementation and Relative Efficiency of Quasirandom
        Sequence Generators,
        ACM Transactions on Mathematical Software,
        Volume 12, Number 4, pages 362-376, 1986.

        Stephen Joe, Frances Kuo
        Remark on Algorithm 659:
        Implementing Sobol's Quasirandom Sequence Generator,
        ACM Transactions on Mathematical Software,
        Volume 29, Number 1, pages 49-57, March 2003.

        Ilya Sobol,
        USSR Computational Mathematics and Mathematical Physics,
        Volume 16, pages 236-242, 1977.

        Ilya Sobol, YL Levitan,
        The Production of Points Uniformly Distributed in a Multidimensional
        Cube (in Russian),
        Preprint IPM Akad. Nauk SSSR,
        Number 40, Moscow 1976.

      Parameters:

                Input, integer DIM_NUM, the spatial dimension.    Only values
                of 1 through 13 will result in useful responses.

                Output, integer TAU, the value TAU.

    """
    dim_max = 13

    tau_table = [0, 0, 1, 3, 5, 8, 11, 15, 19, 23, 27, 31, 35]

    if 1 <= dim_num <= dim_max:
        tau = tau_table[dim_num]
    else:
        tau = -1

    return tau
