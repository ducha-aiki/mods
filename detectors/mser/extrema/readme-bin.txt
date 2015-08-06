Package contains statically linked binary of Maximum Stable Extremal 
Regions detector.

Usage: extrema-bin [options]
Usage: mserdet.exe [options]
    -i (null) [null] input image (png, tiff, jpg, ppm, pgm)
    -o (null) [null] output file
    -es (1.000) [1.000] ellipse scale, (output types 2 and 4)
    -pre (0) [0] image preprocessing type
    -per (0.010) [0.010] maximum relative area
    -ms (30) [30] minimum size of output region
    -mm (10) [10] minimum margin
    -rel (0) [0] use relative margins
    -pe (0) [0] speedup detection by preprocessing local extrema
    -v (0) [0] verbose output
    -d (0) [0] debug outputs
    -r (1) [1] number of runs (for timing purposes)
    -t (0) [0] output file type 0 - RLE, 1 - ExtBound., 2 - Ellipse, 
	       3 - GF, 4 - Aff.
		
    -help (0) [0] print out usage info
Dependencies:
    Option -i is compulsory
Errors detected during  option parsing:
    Missing compulsory option -i

Detected MSER+ and MSER- regions are stored in output file as follows:


Extrema file format RLE:

NUM_MSER_PLUS
NUM_RLE LINE COL1 COL2 LINE COL1 COL2 ... LINE COL1 COL2
...
NUM_RLE LINE COL1 COL2 LINE COL1 COL2 ... LINE COL1 COL2
NUM_MSER_MINUS
NUM_RLE LINE COL1 COL2 LINE COL1 COL2 ... LINE COL1 COL2
...
NUM_RLE LINE COL1 COL2 LINE COL1 COL2 ... LINE COL1 COL2

where NUM_MSER_PLUS and NUM_MSER_MINUS are number of MSER+ and MSER- regions 
respectively. Each region is described as one line in output file. 
NUM_RLE specifies number of RLE triples LINE COL1 COL2.


Extrema file format Extended boundary:

NUM_MSER_PLUS
NUM_PTS X Y X Y ... X Y
...
NUM_PTS X Y X Y ... X Y
NUM_MSER_MINUS
NUM_PTS X Y X Y ... X Y
...
NUM_PTS X Y X Y ... X Y

where NUM_MSER_PLUS and NUM_MSER_MINUS are number of MSER+ and MSER- regions 
respectively. Each region is described as one line in output file. NUM_PTS 
specifies number of points in extended boundary.


Extrema file format Ellipse:

1.0
NUM_TOTAL_MSER
U V A B C 
...
U V A B C

where NUM_TOTAL_MSER is number of MSER+ and MSER- regions. Each region is 
described as one line in output file. Each affine region is described as an 
ellipse with parameters U,V,A,B,C in:
 
a(x-u)(x-u)+2b(x-u)(y-v)+c(y-v)(y-v)=1

with (0,0) at image top left corner.


GF file format is our internal format, for use with an older application (GF 
viewer). The format is self-descriptive, 
(check the header):

num reg_id mini thresh margin min_y min_x area border Pointset

num    		- id of the region
reg_id 		- label of the region
mini   		- minimal intensity
threshold 	- intensity threshold
margin 		- stability (number of intensities that satisfied stability 
		  criterion)
min_y, min_x 	- coordinates of the point with minimal intensity
area, border 	- statistics of the region at the intensity threshold
Pointset 	- list of boundary points (num y0, x0, y1, x1 ...)


Aff file format is another representation of the ellipse of MSER:

NUM_MSER_PLUS
X Y A11 A12 A21 A22 STABILITY
...
NUM_MSER_MINUS
X Y A11 A12 A21 A22 STABILITY
...

where X,Y are the coordinates of the center of the mass. Coeficients 
A11,..,A22 form 2x2 matrix of an affine transformation (together with X,Y) 
that projects points of a unit circle to an ellipse that represents extremal 
region.