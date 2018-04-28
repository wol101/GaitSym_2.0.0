/*
 *  StrokeFont.h
 *  GLCarbon1ContextPbuffer
 *
 *  Created by Bill Sellers on 03/07/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

/* StrokeFont.h - stroke font drawing routines */

/* routine to draw a stroke font character */
/* uses routine DrawLine(ix1,iy1,ix2,iy2) to draw character */

/* cwidth includes a reasonable space between characters, so don't add extra
 space between characters */

/* cheight is maximum cheight above baseline, but characters may descend some
 distance below this (max 3/7 or 0.43), so add c. 50% of cheight between
 lines to prevent overlap */

/* coordinate system assumes origin at bottom left (I think) */

#ifndef StrokeFont_h
#define StrokeFont_h

void StrokeString(
                  char *string,          /* character string */
                  int length,            /* number of characters to draw */
                  double x,              /* x coordinate of bottom left of character */
                  double y,              /* y coordinate ... */
                  double cwidth,         /* cwidth of character cell */
                  double cheight,        /* cheight of character cell */
                  int xJustification,    /* 0 - left, 1 - centre, 2 - right */
                  int yJustification,    /* 0 - bottom, 1 - centre, 2 - top */
                  int orient = 0         /* if non-zero rotate everything 90 degrees AC */);

void StrokeCharacter(
                     int ichar,             /* character code */
                     double x,              /* x coordinate of bottom left of character */
                     double y,              /* y coordinate ... */
                     double cwidth,         /* cwidth of character cell */
                     double cheight,        /* cheight of character cell */
                     int orient = 0         /* if non-zero rotate everything 90 degrees AC */);

#endif
