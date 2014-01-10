/* rgbhashtable.c
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
	int aliceBlue[3];
	int antiqueWhite[3];
	int aqua[3];
	int aquamarine[3];
	int azure[3]; 
	int beige[3];
	int bisque[3]; 
	int black[3]; 
	int blanchedAlmond[3]; 
	int blue[3]; 
	int blueViolet[3]; 
	int brown[3]; 
	int burlyWood[3]; 
	int cadetBlue[3];
	int chartreuse[3];
	int chocolate[3];
	int coral[3]; 
	int cornflowerBlue[3]; 
	int cornSilk[3]; 
	int crimson[3]; 
	int darkBlue[3]; 
	int darkCyan[3]; 
	int darkGoldenrod[3]; 
	int darkGray[3]; 
	int darkGreen[3]; 
	int darkKhaki[3]; 
	int darkMagenta[3]; 
	int darkOliveGreen[3];
	int darkOrange[3]; 
	int darkOrchid[3]; 
	int darkRed[3]; 
	int darkSalmon[3];
	int darkSeaGreen[3]; 
	int darkSlateBlue[3]; 
	int darkSlateGray[3]; 
	int darkTurquoise[3]; 
	int darkViolet[3]; 
	int deepPink[3]; 
	int deepSkyBlue[3]; 
	int dimGray[3];
	int dodgerBlue[3]; 
	int fireBrick[3]; 
	int floralWhite[3]; 
	int forestGreen[3]; 
	int fuchsia[3]; 
	int gainsboro[3]; 
	int ghostWhite[3]; 
	int gold[3]; 
	int goldenrod[3]; 
	int gray[3]; 
	int green[3];
	int greenYellow[3]; 
	int honeydew[3]; 
	int hotPink[3];
	int indianRed[3];
	int indigo[3]; 
	int ivory[3]; 
	int khaki[3]; 
	int lavender[3]; 
	int lavenderBlush[3]; 
	int lawnGreen[3]; 
	int lemonChiffon[3];
	int lightBlue[3]; 
	int lightCoral[3]; 
	int lightCyan[3]; 
	int lightGoldenrodYellow[3]; 
	int lightGray[3];
	int lightGreen[3]; 
	int lightPink[3]; 
	int lightSalmon[3];
    int lightSeaGreen[3]; 
	int lightSkyBlue[3]; 
	int lightSlateGray[3]; 
	int lightSteelBlue[3]; 
	int lightYellow[3]; 
	int limeGreen[3];
    int linen[3]; 
	int maroon[3]; 
	int mediumAquamarine[3]; 
	int mediumBlue[3]; 
	int mediumOrchid[3]; 
	int mediumPurple[3]; 
	int mediumSeaGreen[3]; 
	int mediumSlateBlue[3]; 
	int mediumSpringGreen[3]; 
	int mediumTurquoise[3]; 
	int mediumVioletRed[3]; 
	int midnightBlue[3]; 
	int mintCream[3]; 
	int mistyRose[3];
	int moccasin[3]; 
	int navajoWhite[3]; 
	int navy[3]; 
	int oldLace[3]; 
	int olive[3]; 
	int oliveDrab[3]; 
	int orange[3];
	int orangeRed[3]; 
	int orchid[3]; 
	int paleGoldenrod[3]; 
	int paleGreen[3]; 
	int paleTurquoise[3]; 
	int paleVioletRed[3]; 
	int papayaWhip[3]; 
	int peachPuff[3]; 
	int peru[3]; 
	int pink[3]; 
	int plum[3];
	int powderBlue[3]; 
	int purple[3]; 
	int red[3]; 
	int rosyBrown[3]; 
	int royalBlue[3]; 
	int saddleBrown[3]; 
	int salmon[3]; 
	int sandyBrown[3]; 
	int seaGreen[3]; 
	int seaShell[3]; 
	int sienna[3]; 
	int silver[3]; 
	int skyBlue[3]; 
	int slateBlue[3]; 
	int slateGray[3]; 
	int snow[3]; 
	int springGreen[3];
	int steelBlue[3]; 
	int tan[3]; 
	int teal[3]; 
	int thistle[3]; 
	int tomato[3]; 
	int turquoise[3]; 
	int violet[3]; 
	int wheat[3]; 
	int white[3]; 
	int whiteSmoke[3]; 
	int yellow[3]; 
	int yellowGreen[3]; 
	
  	rgbHashTable * rgbHT = (rgbHashTable *)malloc(sizeof(rgbHashTable));

  	rgbHT->size = MAX_TABLE_SIZE;

  	rgbHT->table = (struct rgbNode **)calloc(1, MAX_TABLE_SIZE * sizeof(struct rgbNode *));

	//Assigning RGB values for each color:
	aliceBlue[0] = 240;
	aliceBlue[1] = 248;
	aliceBlue[2] = 255;
	
	antiqueWhite[0] = 250;
	antiqueWhite[1] = 235;
	antiqueWhite[2] = 215;
	
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
	
	blanchedAlmond[0] = 255;
	blanchedAlmond[1] = 235;
	blanchedAlmond[2] = 205;
  	
	blue[0] = 0;
	blue[1] = 0;
	blue[2] = 255;
	
	blueViolet[0] = 138;
	blueViolet[1] = 43;
	blueViolet[2] = 226;
	
	brown[0] = 165;
	brown[1] = 42;
	brown[2] = 42;
	
	burlyWood[0] = 222;
	burlyWood[1] = 184;
	burlyWood[2] = 135;
	
	cadetBlue[0] = 95;
	cadetBlue[1] = 158;
	cadetBlue[2] = 160;
	
	chartreuse[0] = 127;
	chartreuse[1] = 255;
	chartreuse[2] = 0;
	
	chocolate[0] = 210;
	chocolate[1] = 105;
	chocolate[2] = 30;
	
	coral[0] = 255;
	coral[1] = 127;
	coral[2] = 80;
		
	cornflowerBlue[0] = 100;
	cornflowerBlue[1] = 149;
	cornflowerBlue[2] = 237;
	
	cornSilk[0] = 255;
	cornSilk[1] = 248;
	cornSilk[2] = 220;
	
	crimson[0] = 220;
	crimson[1] = 20;
	crimson[2] = 60;	
	
	darkBlue[0] = 0;
	darkBlue[1] = 0;
	darkBlue[2] = 139;
	
	darkCyan[0] = 0;
	darkCyan[1] = 139;
	darkCyan[2] = 139;
	
	darkGoldenrod[0] = 184;
	darkGoldenrod[1] = 134;
	darkGoldenrod[2] = 11;
	
	darkGray[0] = 169;
	darkGray[1] = 169;
	darkGray[2] = 169;
	
	darkGreen[0] = 0;
	darkGreen[1] = 100;
	darkGreen[2] = 0;
	
	darkKhaki[0] = 189;
	darkKhaki[1] = 183;
	darkKhaki[2] = 107;
	
	darkMagenta[0] = 139;
	darkMagenta[1] = 0; 
	darkMagenta[2] = 139;
	
	darkOliveGreen[0] = 85;
	darkOliveGreen[1] = 107;
	darkOliveGreen[2] = 47;
	
	darkOrange[0] = 255;
	darkOrange[1] = 140;
	darkOrange[2] = 0;
	
	darkOrchid[0] = 153;
	darkOrchid[1] = 50;
	darkOrchid[2] = 204;
	
	darkRed[0] = 139;
	darkRed[1] = 0;
	darkRed[2] = 0;
		 
	darkSalmon[0] = 233;
	darkSalmon[1] = 150;
	darkSalmon[2] = 122; 
	
	darkSeaGreen[0] = 143;
	darkSeaGreen[1] = 188; 
	darkSeaGreen[2] = 143;
	
	darkSlateBlue[0] = 72;
	darkSlateBlue[1] = 61;
	darkSlateBlue[2] = 139;
	
	darkSlateGray[0] = 47;
	darkSlateGray[1] = 79;
	darkSlateGray[2] = 79;
	
	darkTurquoise[0] = 0;
	darkTurquoise[1] = 206;
	darkTurquoise[2] = 209;
	
	darkViolet[0] = 148;
	darkViolet[1] = 0;
	darkViolet[2] = 211;
	
	deepPink[0] = 255;
	deepPink[1] = 20;
	deepPink[2] = 147;
	
	deepSkyBlue[0] = 0;
	deepSkyBlue[1] = 191;
	deepSkyBlue[2] = 255;
	
	dimGray[0] = 105;
	dimGray[1] = 105;
	dimGray[2] = 105;
	
	dodgerBlue[0] = 30;
	dodgerBlue[1] = 144; 
	dodgerBlue[2] = 255;
	
	fireBrick[0] = 178;
	fireBrick[1] = 34; 
	fireBrick[2] = 34;
	
	floralWhite[0] = 255;
	floralWhite[1] = 250;
	floralWhite[2] = 240;
	
	forestGreen[0] = 34;
	forestGreen[1] = 139; 
	forestGreen[2] = 34;
	
	fuchsia[0] = 255;
	fuchsia[1] = 0; 
	fuchsia[2] = 255;
	
	gainsboro[0] = 220;
	gainsboro[1] = 200; 
	gainsboro[2] = 220;
	
	ghostWhite[0] = 248;
	ghostWhite[1] = 248; 
	ghostWhite[2] = 255;
	
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
	
	greenYellow[0] = 173; 
	greenYellow[1] = 255; 
	greenYellow[2] = 47;
	
	honeydew[0] = 240; 
	honeydew[1] = 255; 
	honeydew[2] = 240;
	
	hotPink[0] = 255; 
	hotPink[1] = 105; 
	hotPink[2] = 180;
	
	indianRed[0] = 205;
	indianRed[1] = 92; 
	indianRed[2] = 92;
	
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
	
	lavenderBlush[0] = 255; 
	lavenderBlush[1] = 240; 
	lavenderBlush[2] = 245;
	
	lawnGreen[0] = 124; 
	lawnGreen[1] = 252; 
	lawnGreen[2] = 0;
	
	lemonChiffon[0] = 255; 
	lemonChiffon[1] = 250; 
	lemonChiffon[2] = 205;
	
	lightBlue[0] = 173; 
	lightBlue[1] = 216; 
	lightBlue[2] = 230;
	
	lightCoral[0] = 240; 
	lightCoral[1] = 128; 
	lightCoral[2] = 128;
	
	lightCyan[0] = 224; 
	lightCyan[1] = 255; 
	lightCyan[2] = 255;
	
	lightGoldenrodYellow[0] = 250; 
	lightGoldenrodYellow[1] = 250; 
	lightGoldenrodYellow[2] = 210;
	
	lightGray[0] = 211; 
	lightGray[1] = 211; 
	lightGray[2] = 211;
	
	lightGreen[0] = 144; 
	lightGreen[1] = 238; 
	lightGreen[2] = 144;
	
	lightPink[0] = 255; 
	lightPink[1] = 182; 
	lightPink[2] = 193;
	
	
	lightSalmon[0] = 255; 
	lightSalmon[1] = 160; 
	lightSalmon[2] = 122;
	
	lightSeaGreen[0] = 32; 
	lightSeaGreen[1] = 178; 
	lightSeaGreen[2] = 170;
	
	
	lightSkyBlue[0] = 135; 
	lightSkyBlue[1] = 206; 
	lightSkyBlue[2] = 250;
	
	lightSlateGray[0] = 119; 
	lightSlateGray[1] = 136; 
	lightSlateGray[2] = 153;
	
	lightSteelBlue[0] = 176; 
	lightSteelBlue[1] = 196; 
	lightSteelBlue[2] = 222;
	
	lightYellow[0] = 255; 
	lightYellow[1] = 255; 
	lightYellow[2] = 224;
	
	limeGreen[0] = 50; 
	limeGreen[1] = 205; 
	limeGreen[2] = 50;
	
	linen[0] = 250; 
	linen[1] = 240; 
	linen[2] = 230;
	
	maroon[0] = 128; 
	maroon[1] = 0; 
	maroon[2] = 0;
	
	mediumAquamarine[0] = 102; 
	mediumAquamarine[1] = 205; 
	mediumAquamarine[2] = 170;
	
	mediumBlue[0] = 0; 
	mediumBlue[1] = 0; 
	mediumBlue[2] = 205;
	
	
	mediumOrchid[0] = 186; 
	mediumOrchid[1] = 85; 
	mediumOrchid[2] = 211;
	
	mediumPurple[0] = 147; 
	mediumPurple[1] = 112; 
	mediumPurple[2] = 219;
	
	mediumSeaGreen[0] = 60; 
	mediumSeaGreen[1] = 179; 
	mediumSeaGreen[2] = 113;
	
	mediumSlateBlue[0] = 123, 
	mediumSlateBlue[1] = 104, 
	mediumSlateBlue[2] = 238;
	
	mediumSpringGreen[0] = 0; 
	mediumSpringGreen[1] = 250; 
	mediumSpringGreen[2] = 154;
	
	mediumTurquoise[0] = 72; 
	mediumTurquoise[1] = 209; 
	mediumTurquoise[2] = 204; 
	
	mediumVioletRed[0] = 199; 
	mediumVioletRed[1] = 21; 
	mediumVioletRed[2] = 133;
	
	midnightBlue[0] = 25; 
	midnightBlue[1] = 25; 
	midnightBlue[2] = 112;
	
	mintCream[0] = 245; 
	mintCream[1] = 255; 
	mintCream[2] = 250;
	
	mistyRose[0] = 255, 
	mistyRose[1] = 228, 
	mistyRose[2] = 225;
	
	moccasin[0] = 255; 
	moccasin[1] = 228; 
	moccasin[2] = 181;
	
	navajoWhite[0] = 255; 
	navajoWhite[1] = 222; 
	navajoWhite[2] = 173;
	
	navy[0] = 0; 
	navy[1] = 0; 
	navy[2] = 128;
	
	oldLace[0] = 253; 
	oldLace[1] = 245; 
	oldLace[2] = 230;
	
	olive[0] = 128; 
	olive[1] = 128; 
	olive[2] = 0;
	
	oliveDrab[0] = 107; 
	oliveDrab[1] = 142; 
	oliveDrab[2] = 35;
	
	orange[0] = 255; 
	orange[1] = 165; 
	orange[2] = 0;
	
	orangeRed[0] = 255; 
	orangeRed[1] = 69; 
	orangeRed[2] = 0;
	
	orchid[0] = 218; 
	orchid[1] = 112; 
	orchid[2] = 214;
	
	paleGoldenrod[0] = 238; 
	paleGoldenrod[1] = 232; 
	paleGoldenrod[2] = 170;
	
	paleGreen[0] = 152; 
	paleGreen[1] = 251; 
	paleGreen[2] = 152;
	
	paleTurquoise[0] = 175; 
	paleTurquoise[1] = 238; 
	paleTurquoise[2] = 238;
	
	paleVioletRed[0] = 219, 
	paleVioletRed[1] = 112, 
	paleVioletRed[2] = 147;
	
	papayaWhip[0] = 255, 
	papayaWhip[1] = 239, 
	papayaWhip[2] = 213;
	
	peachPuff[0] = 255; 
	peachPuff[1] = 218; 
	peachPuff[2] = 185;
	
	peru[0] = 205; 
	peru[1] = 133; 
	peru[2] = 63;
	
	pink[0] = 255; 
	pink[1] = 192; 
	pink[2] = 203;
	
	plum[0] = 221; 
	plum[1] = 160; 
	plum[2] = 221;
	
	powderBlue[0] = 176; 
	powderBlue[1] = 224; 
	powderBlue[2] = 230;
	
	purple[0] = 128; 
	purple[1] = 0; 
	purple[2] = 128;
	
	red[0] = 255; 
	red[1] = 0; 
	red[2] = 0;
	
	rosyBrown[0] = 188; 
	rosyBrown[1] = 143; 
	rosyBrown[2] = 143;
	
	royalBlue[0] = 65; 
	royalBlue[1] = 105; 
	royalBlue[2] = 225;
	
	saddleBrown[0] = 139; 
	saddleBrown[1] = 69; 
	saddleBrown[2] = 19;
	
	salmon[0] = 250; 
	salmon[1] = 128; 
	salmon[2] = 114;
	
	sandyBrown[0] = 244; 
	sandyBrown[1] = 164; 
	sandyBrown[2] = 96;
	
	seaGreen[0] = 46; 
	seaGreen[1] = 139; 
	seaGreen[2] = 87;
	
	seaShell[0] = 255; 
	seaShell[1] = 245; 
	seaShell[2] = 238;
	
	sienna[0] = 160; 
	sienna[1] = 82; 
	sienna[2] = 45;
	
	silver[0] = 192; 
	silver[1] = 192; 
	silver[2] = 192;
	
	skyBlue[0] = 135; 
	skyBlue[1] = 206; 
	skyBlue[2] = 235;
	
	slateBlue[0] = 106; 
	slateBlue[1] = 90; 
	slateBlue[2] = 205;
	
	slateGray[0] = 112; 
	slateGray[1] = 128; 
	slateGray[2] = 144;
	
	snow[0] = 255; 
	snow[1] = 250; 
	snow[2] = 250;
	
	springGreen[0] = 0; 
	springGreen[1] = 255; 
	springGreen[2] = 127;
	
	steelBlue[0] = 70; 
	steelBlue[1] = 130; 
	steelBlue[2] = 180;
	
	tan[0] = 210; 
	tan[1] = 180; 
	tan[2] = 140; 
	
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
	
	whiteSmoke[0] = 245; 
	whiteSmoke[1] = 245; 
	whiteSmoke[2] = 245;
	
	yellow[0] = 255; 
	yellow[1] = 255; 
	yellow[2] = 0;
	
	yellowGreen[0] = 154; 
	yellowGreen[1] = 205; 
	yellowGreen[2] = 50;
	
	//Prepopulate the hash table with these colors:
	HT_Add(rgbHT, "aliceBlue", aliceBlue);
	HT_Add(rgbHT, "antiqueWhite", antiqueWhite);
  	HT_Add(rgbHT, "aqua", aqua);
  	HT_Add(rgbHT, "aquamarine", aquamarine);
  	HT_Add(rgbHT, "azure", azure);
  	HT_Add(rgbHT, "beige", beige);
  	HT_Add(rgbHT, "bisque", bisque);
  	HT_Add(rgbHT, "black", black);
  	HT_Add(rgbHT, "blanchedAlmond", blanchedAlmond);
  	HT_Add(rgbHT, "blue", blue);
  	HT_Add(rgbHT, "blueViolet", blueViolet);
  	HT_Add(rgbHT, "brown", brown);
  	HT_Add(rgbHT, "burlyWood", burlyWood);
  	HT_Add(rgbHT, "cadetBlue", cadetBlue);
  	HT_Add(rgbHT, "chartreuse", chartreuse);
  	HT_Add(rgbHT, "chocolate", chocolate);
  	HT_Add(rgbHT, "coral", coral);
  	HT_Add(rgbHT, "cornflowerBlue", cornflowerBlue);
  	HT_Add(rgbHT, "cornSilk", cornSilk);
  	HT_Add(rgbHT, "crimson", crimson);
  	HT_Add(rgbHT, "darkBlue", darkBlue);
  	HT_Add(rgbHT, "darkCyan", darkCyan);
  	HT_Add(rgbHT, "darkGoldenrod", darkGoldenrod);
  	HT_Add(rgbHT, "darkGray", darkGray);
  	HT_Add(rgbHT, "darkKhaki", darkKhaki);
  	HT_Add(rgbHT, "darkGreen", darkGreen);
  	HT_Add(rgbHT, "darkMagenta", darkMagenta);
  	HT_Add(rgbHT, "darkOliveGreen", darkOliveGreen);
  	HT_Add(rgbHT, "darkOrange", darkOrange);
  	HT_Add(rgbHT, "darkOrchid", darkOrchid);
  	HT_Add(rgbHT, "darkRed", darkRed);
  	HT_Add(rgbHT, "darkSalmon", darkSalmon);
  	HT_Add(rgbHT, "darkSeaGreen", darkSeaGreen);
  	HT_Add(rgbHT, "darkSlateBlue", darkSlateBlue);
  	HT_Add(rgbHT, "darkSlateGray", darkSlateGray);
  	HT_Add(rgbHT, "darkTurquoise", darkTurquoise);
  	HT_Add(rgbHT, "darkViolet", darkViolet);
  	HT_Add(rgbHT, "deepPink", deepPink);
  	HT_Add(rgbHT, "deepSkyBlue", deepSkyBlue);
  	HT_Add(rgbHT, "dimGray", dimGray);
  	HT_Add(rgbHT, "dodgerBlue", dodgerBlue);
  	HT_Add(rgbHT, "fireBrick", fireBrick);
  	HT_Add(rgbHT, "floralWhite", floralWhite);
  	HT_Add(rgbHT, "forestGreen", forestGreen);
  	HT_Add(rgbHT, "fuchsia", fuchsia);
  	HT_Add(rgbHT, "gainsboro", gainsboro);
  	HT_Add(rgbHT, "ghostWhite", ghostWhite);
	HT_Add(rgbHT, "gold", gold);
	HT_Add(rgbHT, "goldenrod", goldenrod);
	HT_Add(rgbHT, "gray", gray);
	HT_Add(rgbHT, "green", green);
	HT_Add(rgbHT, "greenYellow", greenYellow);
	HT_Add(rgbHT, "honeydew", honeydew);
	HT_Add(rgbHT, "hotPink", hotPink);
	HT_Add(rgbHT, "indianRed", indianRed);
	HT_Add(rgbHT, "indigo", indigo);
	HT_Add(rgbHT, "ivory", ivory);
	HT_Add(rgbHT, "khaki", khaki);
	HT_Add(rgbHT, "lavender", lavender);
	HT_Add(rgbHT, "lavenderBlush", lavenderBlush);
	HT_Add(rgbHT, "lawnGreen", lawnGreen);
	HT_Add(rgbHT, "lemonChiffon", lemonChiffon);
	HT_Add(rgbHT, "lightBlue", lightBlue);
	HT_Add(rgbHT, "lightCoral", lightCoral);
	HT_Add(rgbHT, "lightCyan", lightCyan);
	HT_Add(rgbHT, "lightGoldenrodYellow", lightGoldenrodYellow);
	HT_Add(rgbHT, "lightGray", lightGray);
	HT_Add(rgbHT, "lightGreen", lightGreen);
	HT_Add(rgbHT, "lightPink", lightPink);
	HT_Add(rgbHT, "lightSalmon", lightSalmon);
	HT_Add(rgbHT, "lightSeaGreen", lightSeaGreen);
	HT_Add(rgbHT, "lightSkyBlue", lightSkyBlue);
	HT_Add(rgbHT, "lightSlateGray", lightSlateGray);
	HT_Add(rgbHT, "lightSteelBlue", lightSteelBlue);
	HT_Add(rgbHT, "lightYellow", lightYellow);
	HT_Add(rgbHT, "limeGreen", limeGreen);
	HT_Add(rgbHT, "linen", linen);
	HT_Add(rgbHT, "maroon", maroon);
	HT_Add(rgbHT, "mediumAquamarine", mediumAquamarine);
	HT_Add(rgbHT, "mediumBlue", mediumBlue);
	HT_Add(rgbHT, "mediumOrchid", mediumOrchid);
	HT_Add(rgbHT, "mediumPurple", mediumPurple);
	HT_Add(rgbHT, "mediumSeaGreen", mediumSeaGreen);
	HT_Add(rgbHT, "mediumSlateBlue", mediumSlateBlue);
	HT_Add(rgbHT, "mediumSpringGreen", mediumSpringGreen);
	HT_Add(rgbHT, "mediumTurquoise", mediumTurquoise);
	HT_Add(rgbHT, "mediumVioletRed", mediumVioletRed);
	HT_Add(rgbHT, "midnightBlue", midnightBlue);
	HT_Add(rgbHT, "mintCream", mintCream);
	HT_Add(rgbHT, "mistyRose", mistyRose);
	HT_Add(rgbHT, "moccasin", moccasin);
	HT_Add(rgbHT, "navajoWhite", navajoWhite);
	HT_Add(rgbHT, "navy", navy);
	HT_Add(rgbHT, "oldLace", oldLace);
	HT_Add(rgbHT, "olive", olive);
	HT_Add(rgbHT, "oliveDrab", oliveDrab);
	HT_Add(rgbHT, "orange", orange);
	HT_Add(rgbHT, "orangeRed", orangeRed);
	HT_Add(rgbHT, "orchid", orchid);
	HT_Add(rgbHT, "paleGoldenrod", paleGoldenrod);
	HT_Add(rgbHT, "paleGreen", paleGreen);
	HT_Add(rgbHT, "paleTurquoise", paleTurquoise);
	HT_Add(rgbHT, "paleVioletRed", paleVioletRed);
	HT_Add(rgbHT, "papayaWhip", papayaWhip);
	HT_Add(rgbHT, "peachPuff", peachPuff);
	HT_Add(rgbHT, "peru", peru);
	HT_Add(rgbHT, "pink", pink);
	HT_Add(rgbHT, "plum", plum);
	HT_Add(rgbHT, "powderBlue", powderBlue);
	HT_Add(rgbHT, "purple", purple);
	HT_Add(rgbHT, "red", red);
	HT_Add(rgbHT, "rosyBrown", rosyBrown);
	HT_Add(rgbHT, "royalBlue", royalBlue);
	HT_Add(rgbHT, "saddleBrown", saddleBrown);
  	HT_Add(rgbHT, "salmon", salmon);
	HT_Add(rgbHT, "sandyBrown", sandyBrown);
	HT_Add(rgbHT, "seaGreen", seaGreen);
	HT_Add(rgbHT, "seaShell", seaShell);
	HT_Add(rgbHT, "sienna", sienna);
  	HT_Add(rgbHT, "silver", silver);
	HT_Add(rgbHT, "skyBlue", skyBlue);
	HT_Add(rgbHT, "slateBlue", slateBlue);
	HT_Add(rgbHT, "slateGray", slateGray);
	HT_Add(rgbHT, "snow", snow);
	HT_Add(rgbHT, "springGreen", springGreen);
	HT_Add(rgbHT, "steelBlue", steelBlue);
	HT_Add(rgbHT, "tan", tan);
  	HT_Add(rgbHT, "teal", teal);
	HT_Add(rgbHT, "thistle", thistle);
	HT_Add(rgbHT, "tomato", tomato);
	HT_Add(rgbHT, "turquoise", turquoise);
	HT_Add(rgbHT, "violet", violet);
	HT_Add(rgbHT, "wheat", wheat);
  	HT_Add(rgbHT, "white", white);
	HT_Add(rgbHT, "whiteSmoke", whiteSmoke);
  	HT_Add(rgbHT, "yellow", yellow);
	HT_Add(rgbHT, "yellowGreen", yellowGreen);

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
