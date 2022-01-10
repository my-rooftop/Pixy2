//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <signal.h>
#include "libpixyusb2.h"
#include <PIDLoop.h>
#include <stdlib.h>
#include <string>
#include <iostream>

using namespace std;
Pixy2 pixy;
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);
PIDLoop rotateLoop(300, 600, 300, false);
PIDLoop translateLoop(400, 800, 300, false);

#define MAX_TRANSLATE_VELOCITY  250
#define PIXY_RCS_MIN_POS                     0
#define PIXY_RCS_MAX_POS                     1000L
#define PIXY_RCS_CENTER_POS                  ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)
static bool  run_flag = true;

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}


// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}

// int writePPM(uint16_t width, uint16_t height, uint32_t *image, const char *filename, int num)
// {
//   int i, j;
//   char fn[32];

//   sprintf(fn, "%s%d.ppm", filename, num);
//   FILE *fp = fopen(fn, "wb");
//   if (fp==NULL)
//     return -1;
//   fprintf(fp, "P6\n%d %d\n255\n", width, height);
//   for (j=0; j<height; j++)
//   {
//     for (i=0; i<width; i++)
//       cout << (char *)(image + j*width + i) << endl;
//       fwrite((char *)(image + j*width + i), 1, 3, fp);
//   }
//   fclose(fp);
//   return 0;
// }

int writePPM(uint16_t width, uint16_t height, uint32_t *image, const char *filename, int num, FILE *fp)
{
  int i, j;

  if (fp==NULL)
    return -1;

  for (j=0; j<height; j++)
  {
    for (i=0; i<width; i++)
      fwrite((char *)(image + j*width + i), 1, 3, fp);
  }
  return 0;
}


int demosaic(uint16_t width, uint16_t height, const uint8_t *bayerImage, uint32_t *image)
{
  uint32_t x, y, xx, yy, r, g, b;
  uint8_t *pixel0, *pixel;
  
  for (y=0; y<height; y++)
  {
    yy = y;
    if (yy==0)
      yy++;
    else if (yy==height-1)
      yy--;
    pixel0 = (uint8_t *)bayerImage + yy*width;
    for (x=0; x<width; x++, image++)
    {
      xx = x;
      if (xx==0)
	xx++;
      else if (xx==width-1)
	xx--;
      pixel = pixel0 + xx;
      if (yy&1)
      {
        if (xx&1)
        {
          r = *pixel;
          g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
          b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
        }
        else
        {
          r = (*(pixel-1)+*(pixel+1))>>1;
          g = *pixel;
          b = (*(pixel-width)+*(pixel+width))>>1;
        }
      }
      else
      {
        if (xx&1)
        {
          r = (*(pixel-width)+*(pixel+width))>>1;
          g = *pixel;
          b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
          r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
          g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
          b = *pixel;
        }
      }
      *image = (b<<16) | (g<<8) | r; 
    }
  }
}


// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i=0; i<pixy.ccc.numBlocks; i++)
  {
    if (index==pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}
const char *array;
string boxy;

int main()
{  
  int i, t  = 0;
  int16_t index=-1;
  char buf[64]; 
  int32_t panOffset, tiltOffset, left, right, pann, tiltt;
  Block *block=NULL;
  // Catch CTRL+C (SIGINT) signals, otherwise the Pixy object
  // won't be cleaned up correctly, leaving Pixy and possibly USB
  // driver in a defunct state.
  signal (SIGINT, handle_SIGINT);
  int  Result;
  uint8_t *bayerFrame;
  uint32_t rgbFrame[PIXY2_RAW_FRAME_WIDTH*PIXY2_RAW_FRAME_HEIGHT];
  // need to initialize pixy!
  pixy.init();
  int count = 0;
  // use ccc program to track objects
  pixy.changeProg("color_connected_components");
 
  FILE* f_rl = fopen("leftandright", "wb");
  FILE* f_in = fopen("input", "wb");
  FILE* f_block = fopen("block", "wb");
  FILE* f_offset = fopen("offset", "wb");
  int turm = 5;

  while(t < 40000)
  {
    t++;
    if (t % turm == 0){
      //pixy.m_link.stop();
      pixy.m_link.getRawFrame(&bayerFrame);
      pixy.m_link.resume();

      demosaic(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, bayerFrame, rgbFrame);
      Result = writePPM(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, rgbFrame, "out", t, f_in);

      if (Result==0)
        printf("Write frame to out.ppm\n");
    }
    pixy.ccc.getBlocks();
    if (index==-1) // search....
    {
      //printf("Searching for block...\n");
      index = acquireBlock();
      if (index>=0)
        printf("Found block!\n");
    }
    // If we've found a block, find it, track it
    if (index>=0)
      block = trackBlock(index);

    if(t % turm == 0){
      cout << "count : " << t <<endl;
      cout << (int)pixy.ccc.numBlocks << endl;
      fwrite(&pixy.ccc.numBlocks, 1, sizeof(uint8_t), f_block);
      for (int Block_Index = 0; Block_Index < pixy.ccc.numBlocks; ++Block_Index)
      {
        //pixy.ccc.blocks[Block_Index].print();
        fwrite(&pixy.ccc.blocks[Block_Index].m_signature, 1, sizeof(uint16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_x, 1, sizeof(uint16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_y, 1, sizeof(uint16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_width, 1, sizeof(uint16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_height, 1, sizeof(uint16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_angle, 1, sizeof(int16_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_index, 1, sizeof(uint8_t), f_block);
        fwrite(&pixy.ccc.blocks[Block_Index].m_age, 1, sizeof(uint8_t), f_block);
      }
    }
    if (block)
    {        
      i++;
      if (i%60==0)
        printf("%d\n", i);   
      
    

      panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)block->m_x;
      tiltOffset = (int32_t)block->m_y - (int32_t)pixy.frameHeight/2;  
  
      panLoop.update(panOffset);
      tiltLoop.update(tiltOffset);
      if (t % turm == 0){
        fwrite(&panOffset, 1, sizeof(int32_t), f_offset);
        fwrite(&tiltOffset, 1, sizeof(int32_t), f_offset);
      }
      pann = panLoop.m_command;
      tiltt = tiltLoop.m_command;
      pixy.setServos(panLoop.m_command, tiltLoop.m_command);//offset <-> m.command graph

      panOffset += panLoop.m_command - PIXY_RCS_CENTER_POS;
      tiltOffset += tiltLoop.m_command - PIXY_RCS_CENTER_POS - PIXY_RCS_CENTER_POS/2 + PIXY_RCS_CENTER_POS/8;
      if (t % turm == 0){
        fwrite(&panOffset, 1, sizeof(int32_t), f_offset);
        fwrite(&tiltOffset, 1, sizeof(int32_t), f_offset);
      }
      rotateLoop.update(panOffset); //offset <-> m.command graph 4 kind
      //make ppt by drawing box... tuesday coding and experiment wednes make ppt
      translateLoop.update(-tiltOffset);

      if (translateLoop.m_command>MAX_TRANSLATE_VELOCITY)
        translateLoop.m_command = MAX_TRANSLATE_VELOCITY;

      left = -rotateLoop.m_command + translateLoop.m_command;
      right = rotateLoop.m_command + translateLoop.m_command;
      //cout << "left: " << left << "right: " << right << endl;
      if (t % turm == 0){
        fwrite(&left, 1, sizeof(int32_t), f_rl);
        fwrite(&right, 1, sizeof(int32_t), f_rl);
        fwrite(&pann, 1, sizeof(int32_t), f_rl);
        fwrite(&tiltt, 1, sizeof(int32_t), f_rl);
      }
    }
    else // no object detected, go into reset state
    {
      panLoop.reset();
      tiltLoop.reset();
      pixy.setServos(panLoop.m_command, tiltLoop.m_command);
      index = -1;
      left = 0;
      right = 0;
      //cout << "left: " << left << "right: " << right << endl;
    }
    boxy = "sudo echo \"" + to_string(left)+" " +to_string(right) + "\" > /dev/ttyACM0";
    //cout << boxy  << endl;
    array = boxy.c_str();
    system(array);

    //collecting data

    if(t % 40000 == 0){
      boxy = "sudo echo \"0 0\" > /dev/ttyACM0";
      //cout << boxy  << endl;
      array = boxy.c_str();
      system(array);
      fclose(f_rl);
      fclose(f_in);
      if (run_flag==false)
      break;

    }
  }
  printf("exiting...\n");
  return 0;

}

