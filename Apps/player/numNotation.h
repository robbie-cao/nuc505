#ifndef __NUM_MUSICAL_NOTATION_H__
#define __NUM_MUSICAL_NOTATION_H__

//#include "mua.h"
/**
 * <b>Numbered Musical Notation</b><br>
 *
 * Reference:
 * - https://en.wikipedia.org/wiki/Numbered_musical_notation
 * - https://en.wikipedia.org/wiki/Clef
 * - https://en.wikipedia.org/wiki/Key_signature
 * - https://en.wikipedia.org/wiki/Time_signature
 *
 */
 #define uint8 unsigned char
 #define uint16 unsigned short

enum {
  CLEF_TYPE_G = 0,
  CLEF_TYPE_C,
  CLEF_TYPE_F,

  CLEF_TYPE_TOTAL
};

enum {
  CLEF_G_TREBLE,
  CLEF_G_FRENCH_VIOLIN
};

enum {
  CLEF_F_BASS,
  CLEF_F_BARITONE,
  CLEF_F_SUBBASS
};

enum {
  CLEF_C_ALTO,
  CLEF_C_TENOR,
  CLEF_C_BARITONE,
  CLEF_C_MEZZON_SOPRANO,
  CLEF_C_SOPRANO
};

enum {
  SCALE_MAJOR,
  SCALE_MINOR,

  SCALE_OCTATONIC ,  ///< (8 notes per octave): used in jazz and modern classical music
  SCALE_HEPTATONIC,  ///< (7 notes per octave): the most common modern Western scale
  SCALE_HEXATONIC ,  ///< (6 notes per octave): common in Western folk music
  SCALE_PENTATONIC,  ///< (5 notes per octave): the anhemitonic form (lacking semitones) is common in folk music, especially in oriental music; also known as the "black note" scale
  SCALE_TETRATONIC,  ///< (4 notes), tritonic (3 notes), and ditonic (2 notes): generally limited to prehistoric ("primitive") music
  SCALE_MONOTONIC ,  ///< (1 note): limited use in liturgy, and for effect in modern art music[citation needed]


  SCALE_TOTAL
};

enum {
  KEY_SIGNATURE_MAJOR_C,
  KEY_SIGNATURE_MAJOR_G,
  KEY_SIGNATURE_MAJOR_D,
  KEY_SIGNATURE_MAJOR_A,
  KEY_SIGNATURE_MAJOR_E,
  KEY_SIGNATURE_MAJOR_B,
  KEY_SIGNATURE_MAJOR_F_SHARP,
  KEY_SIGNATURE_MAJOR_C_SHARP,

  KEY_SIGNATURE_MAJOR_TOTAL
};

enum {
  KEY_SIGNATURE_MINOR_A,
  KEY_SIGNATURE_MINOR_E,
  KEY_SIGNATURE_MINOR_B,
  KEY_SIGNATURE_MINOR_F_SHARP,
  KEY_SIGNATURE_MINOR_C_SHARP,
  KEY_SIGNATURE_MINOR_G_SHARP,
  KEY_SIGNATURE_MINOR_D_SHARP,
  KEY_SIGNATURE_MINOR_A_SHARP,

  KEY_SIGNATURE_MINOR_TOTAL
};

typedef struct {
  uint8 type       : 3;
  uint8 sub        : 3;
  uint8 rfu        : 2;
} Clef_t;

typedef struct {
  uint8 scale      : 3;  // 0 - Major, 1 - Minor
  uint8 signature  : 3;
} KeySignature_t;

/**
 * The time signature is written as a fraction:
 * - 2/4
 * - 3/4
 * - 4/4
 * - 6/8
 * - etc.
 *
 * It is usually placed after the key signature.
 */
typedef struct {
  uint8 upper      : 4;
  uint8 lower      : 2;     ///< 0 - x/1 (NOT USE), 1 - x/2, 2 - x/4, 3 - x/8
} TimeSignature_t;

/*
 * Numbers 1 to 7 represent the musical notes (more accurately the scale degrees).
 */
typedef struct {
  uint8 note       : 3;     ///< 1~7 - do, re, mi, fa, sol, la, si, 0 - rest
  uint8 sharp      : 1;
  uint8 length     : 2;     ///< 0 - half, 1 - quarter, 2 - eighth, 3 - 16th
  uint8 octaves    : 2;     ///< octaves = (-(0b100 - 0bxx) % 0b100), 00 - current, 11 - above +1: 10 - above +2, 01 - below -1

  uint8 oct2       : 1;     ///< extend to 8 octaves
  uint8 dot        : 1;     ///< dot after the plain or underlined note works increases its length by half,
  uint8 expression : 2;     ///< sign or mark used in music to denote a specific expressive quality
  uint8 dynamics   : 2;     ///< p, f, mf, etc
} Note_t;

typedef struct {
  Clef_t            clef;
  KeySignature_t    ks;
  TimeSignature_t   ts;
  uint16            size;
  Note_t*           notes;
} Score_t;

/**
 * Simplified Note
 * - Only support 4 octaves
 * - Not support dot after plain
 */
typedef struct {
  uint8 note       : 3;     ///< 1~7 - do, re, mi, fa, sol, la, si, 0 - rest
  uint8 sharp      : 1;
  uint8 length     : 2;     ///< 0 - half, 1 - quarter, 2 - eighth, 3 - 16th
  uint8 octaves    : 2;     ///< octaves = (-(0b100 - 0bxx) % 0b100), 00 - current, 11 - above +1: 10 - above +2, 01 - below -1
} NoteSimplified_t;

typedef struct {
  Clef_t            clef;
  KeySignature_t    ks;
  TimeSignature_t   ts;
  uint16            size;
  const NoteSimplified_t* notes;
} ScoreSimplified_t;



#endif /* __NUM_MUSICAL_NOTATION_H__ */

/* vim: set ts=2 sw=2 tw=0 list : */
