#ifndef NOISEGENERATOR_H
#define NOISEGENERATOR_H

double gauss_rand (double mean, double dev)
{
  static int iset = 0;
  static double gset;
  double fac, r, v1, v2;

  if (iset)
    { 
      /* We already have a number, so simply return it */
      iset = 0;
      return (gset*dev + mean);
    }
  else
    {
      /* Generate a new random number pair */
      /* Start by getting a random point in the unit circle */
      do {
	/* pick two unifor
(15:39:28) Aude: m numbers in the square [-1, +1] in each direction */
	v1 = 2*(random()/(double) RAND_MAX) -1.0;
	v2 = 2*(random()/(double) RAND_MAX) -1.0;
	r = v1*v1 + v2*v2;     /* See if they fall in the unit circle */
      } while (r >= 1.0);
      
      /* Now do the Box-Muller transformation to get two normal deviates */
      /* return one and save the other */
      fac = sqrt(-2.0*log(r)/r);
      gset = v1*fac;
      iset = 1;
      return (v2*fac*dev + mean);  
    }
}

#endif
