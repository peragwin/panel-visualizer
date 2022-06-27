#pragma once

// Sidebar D - Log Range Reduction Code
/***************************************************
 *
 *  reduce_log2 - The log2 routines require an input argument in
 * the range[0.5, 1.0].This routine reduces the argument to that range.
 *
 * Return values:
 * - "arg", which is the input argument reduced to that range
 * - "n", which is n from the discussion below
 * - "adjustment", set to 1 if the input was < 0.5
 *
 * How this range reduction code works:
 *  If we pick an integer n such that x=f x 2**n, and 0.5<= f < 1, and
 * assume all "log" functions in this discussion means log(base 2),
 * then:
 * log(x) = log(f x 2**n)
 *        = log(f) + log(2**n)
 *        = log(f) + n
 *
 *  The ugly "for" loop shifts a one through two_to_n while shifting a
 * long version of the input argument right, repeating till the long
 * version is all zeroes. Thus, two_to_n and the input argument have
 * this relationship:
 *
 *       two_to_n        input argument           n
 *        1                0.5 to 1               0
 *        2                  1 to 2               1
 *        4                  2 to 4               2
 *        8                  4 to 8               3
 * etc.
 *
 *  There's a special case for the argument being less than 0.5.
 * In this case we use the relation:
 *  log(1/x)=log(1)-log(x)
 *          =      -log(x)
 * That is, we take the reciprocal of x (which will be bigger than
 * 0.5) and solve for the above. To tell the caller to do this, set
 * "adjustment" = 1.
 *
 ***************************************************/
void reduce_log2(float *arg, float *n, int *adjustment)
{
  long two_to_n; // divisor we're looking for: 2^(2k)
  long l_arg;    // long (32 bit) version of input
  *adjustment = 0;
  if (*arg < 0.5)
  { // if small arg use the reciprocal
    *arg = 1.0 / (*arg);
    *adjustment = 1;
  }
  // shift arg to zero while computing two_to_n as described above
  l_arg = (long)*arg; // l_arg = long version of input
  for (two_to_n = 1, *n = 0.0; l_arg != 0; l_arg >>= 1, two_to_n <<= 1, *n += 1.0)
    ;
  *arg = *arg / (float)two_to_n; // normalize argument to [0.5, 1]
};
// Sidebar E: Log base 2 code good to 4.14 digits
/**************************************************
 *
 * LOG2_2521 - compute log(base 2)(x) to 4.14 digits accuracy
 *
 ***************************************************/
float log2_2521(float arg)
{
  const float P00 = -1.45326486;
  const float P01 = +0.951366714;
  const float P02 = +0.501994886;
  const float Q00 = +0.352143751;
  const float Q01 = +1.0;
  float n;        // used to scale result
  float poly;     // result from polynomial
  float answer;   // The result
  int adjustment; // 1 if argument was < 0.5

  //  Return an error if the input is <=0 since log is not real at and
  // below 0.
  if (arg <= 0.0)
  {
    // printf("\nHoly smokes! %d is too durn small. Aborting.", arg);
    return 0;
  }
  reduce_log2(&arg, &n, &adjustment); // reduce  to [0.5, 1.0]
                                      // The format of the polynomial is P(x)/Q(x)
  poly = (P00 + arg * (P01 + arg * P02)) / (Q00 + Q01 * arg);

  // Now correct for the scaling factors
  if (adjustment)
    answer = -n - poly;
  else
    answer = poly + n;
  return (answer);
};
