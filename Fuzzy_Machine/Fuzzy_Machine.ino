// Random fuzzy machine //

#include "Fuzzy_DAC_Audio.h"
#include "sounddata.h"

FuzzyDACAudio audio;

void setup()
{
  
  audio.begin();

}

void loop()
{

  play(rand()%10);

}

void play(uint8_t trackNumber)
{
	switch (trackNumber)
	{
	case 0:
	{
		audio.playHuffArray(HuffDict_Yawing, SoundDataBits_Yawing, SoundData_Yawing);
		break;
	}
	case 1:
	{
		audio.playHuffArray(HuffDict_short_1, SoundDataBits_short_1, SoundData_short_1);
		break;
	}
	case 2:
	{
		audio.playHuffArray(HuffDict_short_2, SoundDataBits_short_2, SoundData_short_2);
		break;
	}
	case 3:
	{
		audio.playHuffArray(HuffDict_short_3, SoundDataBits_short_3, SoundData_short_3);
		break;
	}
	case 4:
	{
		audio.playHuffArray(HuffDict_short_4, SoundDataBits_short_4, SoundData_short_4);
		break;
	}
	case 5:
	{
		audio.playHuffArray(HuffDict_short_5, SoundDataBits_short_5, SoundData_short_5);
		break;
	}
	case 6:
	{
		audio.playHuffArray(HuffDict_short_6, SoundDataBits_short_6, SoundData_short_6);
		break;
	}
	case 7:
	{
		audio.playHuffArray(HuffDict_medium_1, SoundDataBits_medium_1, SoundData_medium_1);
		break;
	}
	case 8:
	{
		audio.playHuffArray(HuffDict_medium_2, SoundDataBits_medium_2, SoundData_medium_2);
		break;
	}
	case 9:
	{
		audio.playHuffArray(HuffDict_medium_3, SoundDataBits_medium_3, SoundData_medium_3);
		break;
	}

	default:
	{
		break;
	}
	}
}
