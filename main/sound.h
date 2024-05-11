#ifndef SOUND_H
#define SOUND_H

/*************************************************************************
Control of the audio amplifier

*****************************************************************************/


void setupSound(int enable_pin);
void play_wav(const char *filename);  



#endif // SOUND_H
