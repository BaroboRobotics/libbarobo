/* rgbhash.c
 *
 * A hashtable for storing RGB values for a Linkbot LED
 * Chaining is used to resolve collisions 
 * RGB values need to be stored as an array of 3 values
 * Includes reverse look-up of a string for nearest color match
 *
 *
 * Dawn Hustig-Schultz
 * 2013/12/20
 *
 */

#include <stdio.h>
//#include <stdint.h>
#include <malloc.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include "rgbhashtable.h"

//The hash function
unsigned long _hash(rgbHashTable * rgbHT, char *key)
{
	unsigned long hash = 5381;

	int i = 0;

	while (hash < ULONG_MAX && i < strlen( key )){
    	hash = (hash << 8) + key[i];
    	i++;
  	}
  	return hash % rgbHT->size;
}

//Create the hash table
rgbHashTable * HT_Create()
{
	//RGB values for the commonly supported browser colors:
	int aliceblue[3];
	int antiquewhite[3];
	int aqua[3];
	int aquamarine[3];
	int azure[3]; 
	int beige[3];
	int bisque[3]; 
	int black[3]; 
	int blanchedalmond[3]; 
	int blue[3]; 
	int blueviolet[3]; 
	int brown[3]; 
	int burlywood[3]; 
	int cadetblue[3];
	int chartreuse[3];
	int chocolate[3];
	int coral[3]; 
	int cornflowerblue[3]; 
	int cornsilk[3]; 
	int crimson[3]; 
	int darkblue[3]; 
	int darkcyan[3]; 
	int darkgoldenrod[3]; 
	int darkgray[3]; 
	int darkgreen[3]; 
	int darkkhaki[3]; 
	int darkmagenta[3]; 
	int darkolivegreen[3];
	int darkorange[3]; 
	int darkorchid[3]; 
	int darkred[3]; 
	int darksalmon[3];
	int darkseagreen[3]; 
	int darkslateblue[3]; 
	int darkslategray[3]; 
	int darkturquoise[3]; 
	int darkviolet[3]; 
	int deeppink[3]; 
	int deepskyblue[3]; 
	int dimgray[3];
	int dodgerblue[3]; 
	int firebrick[3]; 
	int floralwhite[3]; 
	int forestgreen[3]; 
	int fuchsia[3]; 
	int gainsboro[3]; 
	int ghostwhite[3]; 
	int gold[3]; 
	int goldenrod [3]; 
	int gray[3]; 
	int green[3];
	int greenyellow[3]; 
	int honeydew[3]; 
	int hotpink[3];
	int indianred[3];
	int indigo[3]; 
	int ivory[3]; 
	int khaki[3]; 
	int lavender[3]; 
	int lavenderblush[3]; 
	int lawngreen[3]; 
	int lemonchiffon[3];
	int lightblue[3]; 
	int lightcoral[3]; 
	int lightcyan[3]; 
	int lightgoldenrodyellow[3]; 
	int lightgray[3];
	int lightgreen[3]; 
	int lightpink[3]; 
	int lightsalmon[3];
    int lightseagreen[3]; 
	int lightskyblue[3]; 
	int lightslategray[3]; 
	int lightsteelblue[3]; 
	int lightyellow[3]; 
	int limegreen[3];
    int linen[3]; 
	int maroon[3]; 
	int mediumaquamarine[3]; 
	int mediumblue[3]; 
	int mediumorchid[3]; 
	int mediumpurple[3]; 
	int mediumseagreen[3]; 
	int mediumslateblue[3]; 
	int mediumspringgreen[3]; 
	int mediumturquoise[3]; 
	int mediumvioletred[3]; 
	int midnightblue[3]; 
	int mintcream[3]; 
	int mistyrose[3];
	int moccasin[3]; 
	int navajowhite[3]; 
	int navy[3]; 
	int oldlace[3]; 
	int olive[3]; 
	int olivedrab[3]; 
	int orange[3];
	int orangered[3]; 
	int orchid[3]; 
	int palegoldenrod[3]; 
	int palegreen[3]; 
	int paleturquoise[3]; 
	int palevioletred[3]; 
	int papayawhip[3]; 
	int peachpuff[3]; 
	int peru[3]; 
	int pink[3]; 
	int plum[3];
	int powderblue[3]; 
	int purple[3]; 
	int red[3]; 
	int rosybrown[3]; 
	int royalblue[3]; 
	int saddlebrown[3]; 
	int salmon[3]; 
	int sandybrown[3]; 
	int seagreen[3]; 
	int seashell[3]; 
	int sienna[3]; 
	int silver[3]; 
	int skyblue[3]; 
	int slateblue[3]; 
	int slategray[3]; 
	int snow[3]; 
	int springgreen[3];
	int steelblue[3]; 
	int tan[3]; 
	int teal[3]; 
	int thistle[3]; 
	int tomato[3]; 
	int turquoise[3]; 
	int violet[3]; 
	int wheat[3]; 
	int white[3]; 
	int whitesmoke[3]; 
	int yellow[3]; 
	int yellowgreen[3]; 
	
  	rgbHashTable * rgbHT = (rgbHashTable *)malloc(sizeof(rgbHashTable));

  	rgbHT->size = MAX_TABLE_SIZE;

  	rgbHT->table = (struct rgbNode **)calloc(1, MAX_TABLE_SIZE * sizeof(struct rgbNode *));

	//Assigning RGB values for each color:
	aliceblue[0] = 240;
	aliceblue[1] = 248;
	aliceblue[2] = 255;
	
	antiquewhite[0] = 250;
	antiquewhite[1] = 235;
	antiquewhite[2] = 215;
	
	aqua[0] = 0;
	aqua[1] = 255;
	aqua[2] = 255;
	 
	aquamarine[0] = 127;
	aquamarine[1] = 255;
	aquamarine[2] = 212;
	
	azure[0] = 240;
	azure[1] = 255;
	azure[2] = 255;
	 
	beige[0] = 245;
	beige[1] = 245;
	beige[2] = 220;
	
	bisque[0] = 255;
	bisque[1] = 228;
	bisque[2] = 196;
	
	black[0] = 0;
	black[1] = 0;
	black[2] = 0;
	
	blanchedalmond[0] = 255;
	blanchedalmond[1] = 235;
	blanchedalmond[2] = 205;
  	
	blue[0] = 0;
	blue[1] = 0;
	blue[2] = 255;
	
	blueviolet[0] = 138;
	blueviolet[1] = 43;
	blueviolet[2] = 226;
	
	brown[0] = 165;
	brown[1] = 42;
	brown[2] = 42;
	
	burlywood[0] = 222;
	burlywood[1] = 184;
	burlywood[2] = 135;
	
	cadetblue[0] = 95;
	cadetblue[1] = 158;
	cadetblue[2] = 160;
	
	chartreuse[0] = 127;
	chartreuse[1] = 255;
	chartreuse[2] = 0;
	
	chocolate[0] = 210;
	chocolate[1] = 105;
	chocolate[2] = 30;
	
	coral[0] = 255;
	coral[1] = 127;
	coral[2] = 80;
		
	cornflowerblue[0] = 100;
	cornflowerblue[1] = 149;
	cornflowerblue[2] = 237;
	
	cornsilk[0] = 255;
	cornsilk[1] = 248;
	cornsilk[2] = 220;
	
	crimson[0] = 220;
	crimson[1] = 20;
	crimson[2] = 60;	
	
	darkblue[0] = 0;
	darkblue[1] = 0;
	darkblue[2] = 139;
	
	darkcyan[0] = 0;
	darkcyan[1] = 139;
	darkcyan[2] = 139;
	
	darkgoldenrod[0] = 184;
	darkgoldenrod[1] = 134;
	darkgoldenrod[2] = 11;
	
	darkgray[0] = 169;
	darkgray[1] = 169;
	darkgray[2] = 169;
	
	darkgreen[0] = 0;
	darkgreen[1] = 100;
	darkgreen[2] = 0;
	
	darkkhaki[0] = 189;
	darkkhaki[1] = 183;
	darkkhaki[2] = 107;
	
	darkmagenta[0] = 139;
	darkmagenta[1] = 0; 
	darkmagenta[2] = 139;
	
	darkolivegreen[0] = 85;
	darkolivegreen[1] = 107;
	darkolivegreen[2] = 47;
	
	darkorange[0] = 255;
	darkorange[1] = 140;
	darkorange[2] = 0;
	
	darkorchid[0] = 153;
	darkorchid[1] = 50;
	darkorchid[2] = 204;
	
	darkred[0] = 139;
	darkred[1] = 0;
	darkred[2] = 0;
		 
	darksalmon[0] = 233;
	darksalmon[1] = 150;
	darksalmon[2] = 122; 
	
	darkseagreen[0] = 143;
	darkseagreen[1] = 188; 
	darkseagreen[2] = 143;
	
	darkslateblue[0] = 72;
	darkslateblue[1] = 61;
	darkslateblue[2] = 139;
	
	darkslategray[0] = 47;
	darkslategray[1] = 79;
	darkslategray[2] = 79;
	
	darkturquoise[0] = 0;
	darkturquoise[1] = 206;
	darkturquoise[2] = 209;
	
	darkviolet[0] = 148;
	darkviolet[1] = 0;
	darkviolet[2] = 211;
	
	deeppink[0] = 255;
	deeppink[1] = 20;
	deeppink[2] = 147;
	
	deepskyblue[0] = 0;
	deepskyblue[1] = 191;
	deepskyblue[2] = 255;
	
	dimgray[0] = 105;
	dimgray[1] = 105;
	dimgray[2] = 105;
	
	dodgerblue[0] = 30;
	dodgerblue[1] = 144; 
	dodgerblue[2] = 255;
	
	firebrick[0] = 178;
	firebrick[1] = 34; 
	firebrick[2] = 34;
	
	floralwhite[0] = 255;
	floralwhite[1] = 250;
	floralwhite[2] = 240;
	
	forestgreen[0] = 34;
	forestgreen[1] = 139; 
	forestgreen[2] = 34;
	
	fuchsia[0] = 255;
	fuchsia[1] = 0; 
	fuchsia[2] = 255;
	
	gainsboro[0] = 220;
	gainsboro[1] = 200; 
	gainsboro[2] = 220;
	
	ghostwhite[0] = 248;
	ghostwhite[1] = 248; 
	ghostwhite[2] = 255;
	
	gold[0] = 255;
	gold[1] = 215; 
	gold[2] = 0;
	
	goldenrod[0] = 218;
	goldenrod[1] = 165; 
	goldenrod[2] = 32;
	
	gray[0] = 128; 
	gray[1] = 128; 
	gray[2] = 128;
	
	green[0] = 0;
	green[1] = 255; 
	green[2] = 0;
	
	greenyellow[0] = 173; 
	greenyellow[1] = 255; 
	greenyellow[2] = 47;
	
	honeydew[0] = 240; 
	honeydew[1] = 255; 
	honeydew[2] = 240;
	
	hotpink[0] = 255; 
	hotpink[1] = 105; 
	hotpink[2] = 180;
	
	indianred[0] = 205;
	indianred[1] = 92; 
	indianred[2] = 92;
	
	indigo[0] = 75;
	indigo[1] = 0; 
	indigo[2] = 130;
	
	ivory[0] = 255; 
	ivory[1] = 255; 
	ivory[2] = 240;
	
	khaki[0] = 240;
	khaki[1] = 230; 
	khaki[2] = 140;
	
	lavender[0] = 230;
	lavender[1] = 230;
	lavender[2] = 250;
	
	lavenderblush[0] = 255; 
	lavenderblush[1] = 240; 
	lavenderblush[2] = 245;
	
	lawngreen[0] = 124; 
	lawngreen[1] = 252; 
	lawngreen[2] = 0;
	
	lemonchiffon[0] = 255; 
	lemonchiffon[1] = 250; 
	lemonchiffon[2] = 205;
	
	lightblue[0] = 173; 
	lightblue[1] = 216; 
	lightblue[2] = 230;
	
	lightcoral[0] = 240; 
	lightcoral[1] = 128; 
	lightcoral[2] = 128;
	
	lightcyan[0] = 224; 
	lightcyan[1] = 255; 
	lightcyan[2] = 255;
	
	lightgoldenrodyellow[0] = 250; 
	lightgoldenrodyellow[1] = 250; 
	lightgoldenrodyellow[2] = 210;
	
	lightgray[0] = 211; 
	lightgray[1] = 211; 
	lightgray[2] = 211;
	
	lightgreen[0] = 144; 
	lightgreen[1] = 238; 
	lightgreen[2] = 144;
	
	lightpink[0] = 255; 
	lightpink[1] = 182; 
	lightpink[2] = 193;
	
	
	lightsalmon[0] = 255; 
	lightsalmon[1] = 160; 
	lightsalmon[2] = 122;
	
	lightseagreen[0] = 32; 
	lightseagreen[1] = 178; 
	lightseagreen[2] = 170;
	
	
	lightskyblue[0] = 135; 
	lightskyblue[1] = 206; 
	lightskyblue[2] = 250;
	
	lightslategray[0] = 119; 
	lightslategray[1] = 136; 
	lightslategray[2] = 153;
	
	lightsteelblue[0] = 176; 
	lightsteelblue[1] = 196; 
	lightsteelblue[2] = 222;
	
	lightyellow[0] = 255; 
	lightyellow[1] = 255; 
	lightyellow[2] = 224;
	
	limegreen[0] = 50; 
	limegreen[1] = 205; 
	limegreen[2] = 50;
	
	linen[0] = 250; 
	linen[1] = 240; 
	linen[2] = 230;
	
	maroon[0] = 128; 
	maroon[1] = 0; 
	maroon[2] = 0;
	
	mediumaquamarine[0] = 102; 
	mediumaquamarine[1] = 205; 
	mediumaquamarine[2] = 170;
	
	mediumblue[0] = 0; 
	mediumblue[1] = 0; 
	mediumblue[2] = 205;
	
	
	mediumorchid[0] = 186; 
	mediumorchid[1] = 85; 
	mediumorchid[2] = 211;
	
	mediumpurple[0] = 147; 
	mediumpurple[1] = 112; 
	mediumpurple[2] = 219;
	
	mediumseagreen[0] = 60; 
	mediumseagreen[1] = 179; 
	mediumseagreen[2] = 113;
	
	mediumslateblue[0] = 123, 
	mediumslateblue[1] = 104, 
	mediumslateblue[2] = 238;
	
	mediumspringgreen[0] = 0; 
	mediumspringgreen[1] = 250; 
	mediumspringgreen[2] = 154;
	
	mediumturquoise[0] = 72; 
	mediumturquoise[1] = 209; 
	mediumturquoise[2] = 204; 
	
	mediumvioletred[0] = 199; 
	mediumvioletred[1] = 21; 
	mediumvioletred[2] = 133;
	
	midnightblue[0] = 25; 
	midnightblue[1] = 25; 
	midnightblue[2] = 112;
	
	mintcream[0] = 245; 
	mintcream[1] = 255; 
	mintcream[2] = 250;
	
	mistyrose[0] = 255, 
	mistyrose[1] = 228, 
	mistyrose[2] = 225;
	
	moccasin[0] = 255; 
	moccasin[1] = 228; 
	moccasin[2] = 181;
	
	navajowhite[0] = 255; 
	navajowhite[1] = 222; 
	navajowhite[2] = 173;
	
	navy[0] = 0; 
	navy[1] = 0; 
	navy[2] = 128;
	
	oldlace[0] = 253; 
	oldlace[1] = 245; 
	oldlace[2] = 230;
	
	olive[0] = 128; 
	olive[1] = 128; 
	olive[2] = 0;
	
	olivedrab[0] = 107; 
	olivedrab[1] = 142; 
	olivedrab[2] = 35;
	
	orange[0] = 255; 
	orange[1] = 165; 
	orange[2] = 0;
	
	orangered[0] = 255; 
	orangered[1] = 69; 
	orangered[2] = 0;
	
	orchid[0] = 218; 
	orchid[1] = 112; 
	orchid[2] = 214;
	
	palegoldenrod[0] = 238; 
	palegoldenrod[1] = 232; 
	palegoldenrod[2] = 170;
	
	palegreen[0] = 152; 
	palegreen[1] = 251; 
	palegreen[2] = 152;
	
	paleturquoise[0] = 175; 
	paleturquoise[1] = 238; 
	paleturquoise[2] = 238;
	
	palevioletred[0] = 219, 
	palevioletred[1] = 112, 
	palevioletred[2] = 147;
	
	papayawhip[0] = 255, 
	papayawhip[1] = 239, 
	papayawhip[2] = 213;
	
	peachpuff[0] = 255; 
	peachpuff[1] = 218; 
	peachpuff[2] = 185;
	
	peru[0] = 205; 
	peru[1] = 133; 
	peru[2] = 63;
	
	pink[0] = 255; 
	pink[1] = 192; 
	pink[2] = 203;
	
	plum[0] = 221; 
	plum[1] = 160; 
	plum[2] = 221;
	
	powderblue[0] = 176; 
	powderblue[1] = 224; 
	powderblue[2] = 230;
	
	purple[0] = 128; 
	purple[1] = 0; 
	purple[2] = 128;
	
	red[0] = 255; 
	red[1] = 0; 
	red[2] = 0;
	
	rosybrown[0] = 188; 
	rosybrown[1] = 143; 
	rosybrown[2] = 143;
	
	royalblue[0] = 65; 
	royalblue[1] = 105; 
	royalblue[2] = 225;
	
	saddlebrown[0] = 139; 
	saddlebrown[1] = 69; 
	saddlebrown[2] = 19;
	
	salmon[0] = 250; 
	salmon[1] = 128; 
	salmon[2] = 114;
	
	sandybrown[0] = 244; 
	sandybrown[1] = 164; 
	sandybrown[2] = 96;
	
	seagreen[0] = 46; 
	seagreen[1] = 139; 
	seagreen[2] = 87;
	
	seashell[0] = 255; 
	seashell[1] = 245; 
	seashell[2] = 238;
	
	sienna[0] = 160; 
	sienna[1] = 82; 
	sienna[2] = 45;
	
	silver[0] = 192; 
	silver[1] = 192; 
	silver[2] = 192;
	
	skyblue[0] = 135; 
	skyblue[1] = 206; 
	skyblue[2] = 235;
	
	slateblue[0] = 106; 
	slateblue[1] = 90; 
	slateblue[2] = 205;
	
	slategray[0] = 112; 
	slategray[1] = 128; 
	slategray[2] = 144;
	
	snow[0] = 255; 
	snow[1] = 250; 
	snow[2] = 250;
	
	springgreen[0] = 0; 
	springgreen[1] = 255; 
	springgreen[2] = 127;
	
	steelblue[0] = 70; 
	steelblue[1] = 130; 
	steelblue[2] = 180;
	
	steelblue[0] = 210; 
	steelblue[1] = 180; 
	steelblue[2] = 140; 
	
	teal[0] = 0; 
	teal[1] = 128; 
	teal[2] = 128;
	
	thistle[0] = 216; 
	thistle[1] = 191; 
	thistle[2] = 216;
	
	tomato[0] = 255; 
	tomato[1] = 99; 
	tomato[2] = 71;
	
	turquoise[0] = 64; 
	turquoise[1] = 224; 
	turquoise[2] = 208;
	
	violet[0] = 238; 
	violet[1] = 130; 
	violet[2] = 238;
	
	wheat[0] = 245; 
	wheat[1] = 222; 
	wheat[2] = 179;
	
	white[0] = 255; 
	white[1] = 255; 
	white[2] = 255;
	
	whitesmoke[0] = 245; 
	whitesmoke[1] = 245; 
	whitesmoke[2] = 245;
	
	yellow[0] = 255; 
	yellow[1] = 255; 
	yellow[2] = 0;
	
	yellowgreen[0] = 154; 
	yellowgreen[1] = 205; 
	yellowgreen[2] = 50;
	
	//Prepopulate the hash table with these colors:
	HT_Add(rgbHT, "aliceblue", aliceblue);
	HT_Add(rgbHT, "antiquewhite", antiquewhite);
  	HT_Add(rgbHT, "aqua", aqua);
  	HT_Add(rgbHT, "aquamarine", aquamarine);
  	HT_Add(rgbHT, "azure", azure);
  	HT_Add(rgbHT, "beige", beige);
  	HT_Add(rgbHT, "bisque", bisque);
  	HT_Add(rgbHT, "black", black);
  	HT_Add(rgbHT, "blanchedalmond", blanchedalmond);
  	HT_Add(rgbHT, "blue", blue);
  	HT_Add(rgbHT, "blueviolet", blueviolet);
  	HT_Add(rgbHT, "brown", brown);
  	HT_Add(rgbHT, "burlywood", burlywood);
  	HT_Add(rgbHT, "cadetblue", cadetblue);
  	HT_Add(rgbHT, "chartreuse", chartreuse);
  	HT_Add(rgbHT, "chocolate", chocolate);
  	HT_Add(rgbHT, "coral", coral);
  	HT_Add(rgbHT, "cornflowerblue", cornflowerblue);
  	HT_Add(rgbHT, "cornsilk", cornsilk);
  	HT_Add(rgbHT, "crimson", crimson);
  	HT_Add(rgbHT, "darkblue", darkblue);
  	HT_Add(rgbHT, "darkcyan", darkcyan);
  	HT_Add(rgbHT, "darkgoldenrod", darkgoldenrod);
  	HT_Add(rgbHT, "darkgray", darkgray);
  	HT_Add(rgbHT, "darkkhaki", darkkhaki);
  	HT_Add(rgbHT, "darkgreen", darkgreen);
  	HT_Add(rgbHT, "darkmagenta", darkmagenta);
  	HT_Add(rgbHT, "darkolivegreen", darkolivegreen);
  	HT_Add(rgbHT, "darkorange", darkorange);
  	HT_Add(rgbHT, "darkorchid", darkorchid);
  	HT_Add(rgbHT, "darkred", darkred);
  	HT_Add(rgbHT, "darksalmon", darksalmon);
  	HT_Add(rgbHT, "darkseagreen", darkseagreen);
  	HT_Add(rgbHT, "darkslateblue", darkslateblue);
  	HT_Add(rgbHT, "darkturquoise", darkturquoise);
  	HT_Add(rgbHT, "darkslategray", darkslategray);
  	HT_Add(rgbHT, "darkviolet", darkviolet);
  	HT_Add(rgbHT, "deeppink", deeppink);
  	HT_Add(rgbHT, "deepskyblue", deepskyblue);
  	HT_Add(rgbHT, "dimgray", dimgray);
  	HT_Add(rgbHT, "dodgerblue", dodgerblue);
  	HT_Add(rgbHT, "firebrick", firebrick);
  	HT_Add(rgbHT, "floralwhite", floralwhite);
  	HT_Add(rgbHT, "forestgreen", forestgreen);
  	HT_Add(rgbHT, "fuchsia", fuchsia);
  	HT_Add(rgbHT, "gainsboro", gainsboro);
  	HT_Add(rgbHT, "ghostwhite", ghostwhite);
	HT_Add(rgbHT, "gold", gold);
	HT_Add(rgbHT, "goldenrod", goldenrod);
	HT_Add(rgbHT, "gray", gray);
	HT_Add(rgbHT, "green", green);
	HT_Add(rgbHT, "greenyellow", greenyellow);
	HT_Add(rgbHT, "honeydew", honeydew);
	HT_Add(rgbHT, "hotpink", hotpink);
	HT_Add(rgbHT, "indianred", indianred);
	HT_Add(rgbHT, "indigo", indigo);
	HT_Add(rgbHT, "ivory", ivory);
	HT_Add(rgbHT, "khaki", khaki);
	HT_Add(rgbHT, "lavender", lavender);
	HT_Add(rgbHT, "lavenderblush", lavenderblush);
	HT_Add(rgbHT, "lawngreen", lawngreen);
	HT_Add(rgbHT, "lemonchiffon", lemonchiffon);
	HT_Add(rgbHT, "lightblue", lightblue);
	HT_Add(rgbHT, "lightcoral", lightcoral);
	HT_Add(rgbHT, "lightcyan", lightcyan);
	HT_Add(rgbHT, "lightgoldenrodyellow", lightgoldenrodyellow);
	HT_Add(rgbHT, "lightgray", lightgray);
	HT_Add(rgbHT, "lightgreen", lightgreen);
	HT_Add(rgbHT, "lightpink", lightpink);
	HT_Add(rgbHT, "lightsalmon", lightsalmon);
	HT_Add(rgbHT, "lightseagreen", lightseagreen);
	HT_Add(rgbHT, "lightskyblue", lightskyblue);
	HT_Add(rgbHT, "lightslategray", lightslategray);
	HT_Add(rgbHT, "lightsteelblue", lightsteelblue);
	HT_Add(rgbHT, "lightyellow", lightyellow);
	HT_Add(rgbHT, "limegreen", limegreen);
	HT_Add(rgbHT, "linen", linen);
	HT_Add(rgbHT, "maroon", maroon);
	HT_Add(rgbHT, "mediumaquamarine", mediumaquamarine);
	HT_Add(rgbHT, "mediumblue", mediumblue);
	HT_Add(rgbHT, "mediumorchid", mediumorchid);
	HT_Add(rgbHT, "mediumpurple", mediumpurple);
	HT_Add(rgbHT, "mediumseagreen", mediumseagreen);
	HT_Add(rgbHT, "mediumslateblue", mediumslateblue);
	HT_Add(rgbHT, "mediumspringgreen", mediumspringgreen);
	HT_Add(rgbHT, "mediumturquoise", mediumturquoise);
	HT_Add(rgbHT, "mediumvioletred", mediumvioletred);
	HT_Add(rgbHT, "midnightblue", midnightblue);
	HT_Add(rgbHT, "mintcream", mintcream);
	HT_Add(rgbHT, "mistyrose", mistyrose);
	HT_Add(rgbHT, "moccasin", moccasin);
	HT_Add(rgbHT, "navajowhite", navajowhite);
	HT_Add(rgbHT, "navy", navy);
	HT_Add(rgbHT, "oldlace", oldlace);
	HT_Add(rgbHT, "olive", olive);
	HT_Add(rgbHT, "olivedrab", olivedrab);
	HT_Add(rgbHT, "orange", orange);
	HT_Add(rgbHT, "orangered", orangered);
	HT_Add(rgbHT, "orchid", orchid);
	HT_Add(rgbHT, "palegoldenrod", palegoldenrod);
	HT_Add(rgbHT, "palegreen", palegreen);
	HT_Add(rgbHT, "paleturquoise", paleturquoise);
	HT_Add(rgbHT, "palevioletred", palevioletred);
	HT_Add(rgbHT, "papayawhip", papayawhip);
	HT_Add(rgbHT, "peachpuff", peachpuff);
	HT_Add(rgbHT, "peru", peru);
	HT_Add(rgbHT, "pink", pink);
	HT_Add(rgbHT, "plum", plum);
	HT_Add(rgbHT, "powderblue", powderblue);
	HT_Add(rgbHT, "purple", purple);
	HT_Add(rgbHT, "red", red);
	HT_Add(rgbHT, "rosybrown", rosybrown);
	HT_Add(rgbHT, "royalblue", royalblue);
	HT_Add(rgbHT, "saddlebrown", saddlebrown);
  	HT_Add(rgbHT, "salmon", salmon);
	HT_Add(rgbHT, "sandybrown", sandybrown);
	HT_Add(rgbHT, "seagreen", seagreen);
	HT_Add(rgbHT, "seashell", seashell);
	HT_Add(rgbHT, "sienna", sienna);
  	HT_Add(rgbHT, "silver", silver);
	HT_Add(rgbHT, "skyblue", skyblue);
	HT_Add(rgbHT, "slateblue", slateblue);
	HT_Add(rgbHT, "slategray", slategray);
	HT_Add(rgbHT, "snow", snow);
	HT_Add(rgbHT, "springgreen", springgreen);
	HT_Add(rgbHT, "steelblue", steelblue);
	HT_Add(rgbHT, "tan", tan);
  	HT_Add(rgbHT, "teal", teal);
	HT_Add(rgbHT, "thistle", thistle);
	HT_Add(rgbHT, "tomato", tomato);
	HT_Add(rgbHT, "turquoise", turquoise);
	HT_Add(rgbHT, "violet", violet);
	HT_Add(rgbHT, "wheat", wheat);
  	HT_Add(rgbHT, "white", white);
	HT_Add(rgbHT, "whitesmoke", whitesmoke);
  	HT_Add(rgbHT, "yellow", yellow);
	HT_Add(rgbHT, "yellowgreen", yellowgreen);

  	return rgbHT;
}

//Destroy the table
void HT_Destroy(rgbHashTable * rgbHT)
{
	int i;
	
  	if (!rgbHT) return;

  	for (i = 0; i < rgbHT->size; i++) {

    	struct rgbNode * n = rgbHT->table[i];

    	while (n) {
     		struct rgbNode *n_old = n;

      		n = n->next;               

      		free(n_old->key);

      		n_old->key = NULL;

      		free(n_old);

      		n_old = NULL;
    	}
  	}

  	free(rgbHT->table);

  	free(rgbHT);

  	rgbHT = NULL;
}

//Pass in a key, retrieve a set of RGB values
int HT_Get(rgbHashTable * rgbHT, char * key, int * rgbArray)
{
	unsigned long index;
	struct rgbNode * n = NULL;

  	if (!rgbHT) return -1; //If not a valid hash table, then return negative value

  	index = _hash(rgbHT, key);

  	n = rgbHT->table[index];

  	while (n) {
    	if (strncmp(key, n->key, MAX_KEYLEN) == 0){
      		rgbArray[0] = n->values[0];
			rgbArray[1] = n->values[1];
			rgbArray[2] = n->values[2];
			return 1;	//If the entry exists, return true
		}
    	n = n->next;
  	}
	return -1;	//If get to this point, entry not in table. Return negative value.
}



//Store a set of RGB values with a specified key
void HT_Add(rgbHashTable * rgbHT, char * key, int values[])
{
	unsigned long index;
	struct rgbNode *n_new = NULL;
	int length;
	
  	if (!rgbHT) return;

  	index = _hash(rgbHT, key);

  	n_new = (struct rgbNode*)calloc(1, sizeof(struct rgbNode));

  	n_new->values[0] = values[0]; 
  	n_new->values[1] = values[1];
  	n_new->values[2] = values[2];
	
	length = strlen(key);
	if (length > MAX_KEYLEN){
		length = MAX_KEYLEN;
	}
	
  	n_new->key = (char*)calloc(1, length + 1);

  	strcpy(n_new->key, key);

  	n_new->next = rgbHT->table[index];
  	rgbHT->table[index] = n_new;
}

//Remove a set of RGB values from the table
void HT_Remove(rgbHashTable *rgbHT, char *key)
{
	unsigned long index;
	struct rgbNode *p = NULL;
	struct rgbNode *n = NULL;

  	if (!rgbHT) return;

  	index = _hash(rgbHT, key);    

  	n = rgbHT->table[index];

  	while (n) {
    	if (strncmp(key, n->key, MAX_KEYLEN) == 0) {
      		if (p)
        		p->next = n->next;


      	free (n->key);
      	n->key = NULL;          

      	if (rgbHT->table[index] == n)
        	rgbHT->table[index] = NULL;
    

      	free (n);
      	n = NULL;         

      	break;

    	}          

    	p = n;
    	n = n->next;

  	}
}

//match a set of RGB values with the closest table entry. Then copy this entry's key into "color": 
int HT_GetKey(rgbHashTable * rgbHT, int values[], char color[])
{
	int i;
	double distance;
	double shortestDistance;
	char * shortestKey = NULL;
	
	if (!rgbHT) return -1;
	shortestDistance = 3 * pow((double)255, 2); //Start with max distance, which is the distance between black and white.

	for (i = 0; i < rgbHT->size; i++) {
		struct rgbNode * n = rgbHT->table[i];

		while(n) {
			//Compute square of distance between the RGB values from the LED and the RGB values of the current table entry:
			distance = pow((double)(values[0] - n->values[0]), 2) + pow((double)(values[1] - n->values[1]), 2) + pow((double)(values[2] - n->values[2]), 2);
			//Compare with previously found shortest distance. If current distance is shorter, then update:
			if (distance < shortestDistance){ 
				
				shortestDistance = distance;
				//keep track of key:
				shortestKey = n->key;
			}
			n = n->next;
		}
	}
	strcpy(color, shortestKey);
	return 0;
}
