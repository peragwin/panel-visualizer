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
void reduce_log2(float *arg, float *n, int *adjustment);
// Sidebar E: Log base 2 code good to 4.14 digits
/**************************************************
 *
 * LOG2_2521 - compute log(base 2)(x) to 4.14 digits accuracy
 *
 ***************************************************/
float log2_2521(float arg);
