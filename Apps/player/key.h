#ifndef __KEY_H__
#define __KEY_H__

/*============================================================================
 * Include files
 */
//#include "hal_types.h"

/*============================================================================
 * Macros
 */
#define KEY_FILTER        1


/*============================================================================
 * Enums
 */
/**
 * Key Map
 */
enum {
  /*!< Standard Piano Key, Start from 21 and end at 108 */
  /**
   * <br>
   * There are 128 possible notes on a MIDI device, numbered 0 to 127
   * (where Middle C is note number 60).
   */
                        /* 0~20 Reserved */
  KEY_PIANO_BEGIN = 21 ,
  KEY_PIANO_U2_A      =   21 ,
  KEY_PIANO_U2_AS     =   22 ,
  KEY_PIANO_U2_BB     =   22 ,
  KEY_PIANO_U2_B      =   23 ,

  KEY_PIANO_U1_C      =   24 ,
  KEY_PIANO_U1_CS     =   25 ,
  KEY_PIANO_U1_DB     =   25 ,
  KEY_PIANO_U1_D      =   26 ,
  KEY_PIANO_U1_DS     =   27 ,
  KEY_PIANO_U1_EB     =   27 ,
  KEY_PIANO_U1_E      =   28 ,
  KEY_PIANO_U1_F      =   29 ,
  KEY_PIANO_U1_FS     =   30 ,
  KEY_PIANO_U1_GB     =   30 ,
  KEY_PIANO_U1_G      =   31 ,
  KEY_PIANO_U1_GS     =   32 ,
  KEY_PIANO_U1_AB     =   32 ,
  KEY_PIANO_U1_A      =   33 ,
  KEY_PIANO_U1_AS     =   34 ,
  KEY_PIANO_U1_BB     =   34 ,
  KEY_PIANO_U1_B      =   35 ,

  KEY_PIANO_U0_C      =   36 ,
  KEY_PIANO_U0_CS     =   37 ,
  KEY_PIANO_U0_DB     =   37 ,
  KEY_PIANO_U0_D      =   38 ,
  KEY_PIANO_U0_DS     =   39 ,
  KEY_PIANO_U0_EB     =   39 ,
  KEY_PIANO_U0_E      =   40 ,
  KEY_PIANO_U0_F      =   41 ,
  KEY_PIANO_U0_FS     =   42 ,
  KEY_PIANO_U0_GB     =   42 ,
  KEY_PIANO_U0_G      =   43 ,
  KEY_PIANO_U0_GS     =   44 ,
  KEY_PIANO_U0_AB     =   44 ,
  KEY_PIANO_U0_A      =   45 ,
  KEY_PIANO_U0_AS     =   46 ,
  KEY_PIANO_U0_BB     =   46 ,
  KEY_PIANO_U0_B      =   47 ,

  KEY_PIANO_L0_C      =   48 ,
  KEY_PIANO_L0_CS     =   49 ,
  KEY_PIANO_L0_DB     =   49 ,
  KEY_PIANO_L0_D      =   50 ,
  KEY_PIANO_L0_DS     =   51 ,
  KEY_PIANO_L0_EB     =   51 ,
  KEY_PIANO_L0_E      =   52 ,
  KEY_PIANO_L0_F      =   53 ,
  KEY_PIANO_L0_FS     =   54 ,
  KEY_PIANO_L0_GB     =   54 ,
  KEY_PIANO_L0_G      =   55 ,
  KEY_PIANO_L0_GS     =   56 ,
  KEY_PIANO_L0_AB     =   56 ,
  KEY_PIANO_L0_A      =   57 ,
  KEY_PIANO_L0_AS     =   58 ,
  KEY_PIANO_L0_BB     =   58 ,
  KEY_PIANO_L0_B      =   59 ,

  KEY_PIANO_L1_C      =   60 ,
  KEY_PIANO_L1_CS     =   61 ,
  KEY_PIANO_L1_DB     =   61 ,
  KEY_PIANO_L1_D      =   62 ,
  KEY_PIANO_L1_DS     =   63 ,
  KEY_PIANO_L1_EB     =   63 ,
  KEY_PIANO_L1_E      =   64 ,
  KEY_PIANO_L1_F      =   65 ,
  KEY_PIANO_L1_GB     =   66 ,
  KEY_PIANO_L1_G      =   67 ,
  KEY_PIANO_L1_GS     =   68 ,
  KEY_PIANO_L1_AB     =   68 ,
  KEY_PIANO_L1_A      =   69 ,
  KEY_PIANO_L1_AS     =   70 ,
  KEY_PIANO_L1_BB     =   70 ,
  KEY_PIANO_L1_B      =   71 ,

  KEY_PIANO_L2_C      =   72 ,
  KEY_PIANO_L2_CS     =   73 ,
  KEY_PIANO_L2_DB     =   73 ,
  KEY_PIANO_L2_D      =   74 ,
  KEY_PIANO_L2_DS     =   75 ,
  KEY_PIANO_L2_EB     =   75 ,
  KEY_PIANO_L2_E      =   76 ,
  KEY_PIANO_L2_F      =   77 ,
  KEY_PIANO_L2_FS     =   78 ,
  KEY_PIANO_L2_GB     =   78 ,
  KEY_PIANO_L2_G      =   79 ,
  KEY_PIANO_L2_GS     =   80 ,
  KEY_PIANO_L2_AB     =   80 ,
  KEY_PIANO_L2_A      =   81 ,
  KEY_PIANO_L2_AS     =   82 ,
  KEY_PIANO_L2_BB     =   82 ,
  KEY_PIANO_L2_B      =   83 ,

  KEY_PIANO_L3_C      =   84 ,
  KEY_PIANO_L3_CS     =   85 ,
  KEY_PIANO_L3_DB     =   85 ,
  KEY_PIANO_L3_D      =   86 ,
  KEY_PIANO_L3_DS     =   87 ,
  KEY_PIANO_L3_EB     =   87 ,
  KEY_PIANO_L3_E      =   88 ,
  KEY_PIANO_L3_F      =   89 ,
  KEY_PIANO_L3_FS     =   90 ,
  KEY_PIANO_L3_GB     =   90 ,
  KEY_PIANO_L3_G      =   91 ,
  KEY_PIANO_L3_GS     =   92 ,
  KEY_PIANO_L3_AB     =   92 ,
  KEY_PIANO_L3_A      =   93 ,
  KEY_PIANO_L3_AS     =   94 ,
  KEY_PIANO_L3_BB     =   94 ,
  KEY_PIANO_L3_B      =   95 ,

  KEY_PIANO_L4_C      =   96 ,
  KEY_PIANO_L4_CS     =   97 ,
  KEY_PIANO_L4_DB     =   97 ,
  KEY_PIANO_L4_D      =   98 ,
  KEY_PIANO_L4_DS     =   99 ,
  KEY_PIANO_L4_EB     =   99 ,
  KEY_PIANO_L4_E      =  100 ,
  KEY_PIANO_L4_F      =  101 ,
  KEY_PIANO_L4_FS     =  102 ,
  KEY_PIANO_L4_GB     =  102 ,
  KEY_PIANO_L4_G      =  103 ,
  KEY_PIANO_L4_GS     =  104 ,
  KEY_PIANO_L4_AB     =  104 ,
  KEY_PIANO_L4_A      =  105 ,
  KEY_PIANO_L4_AS     =  106 ,
  KEY_PIANO_L4_BB     =  106 ,
  KEY_PIANO_L4_B      =  107 ,

  KEY_PIANO_L5_C      =  108 ,
  KEY_PIANO_END = 108  ,
                        /* 109~127 Reserved */
  /*!< Navigation Key */
  KEY_NAV_BEGIN = 128,
  KEY_NAV_UP = 128,
  KEY_NAV_DOWN,
  KEY_NAV_LEFT,
  KEY_NAV_RIGHT,
  KEY_NAV_FIRST,
  KEY_NAV_LAST,
  // Add more TBD here if any
  KEY_NAV_END = 135,

  KEY_VOL_BEGIN = 136,
  KEY_VOL_UP = 136,
  KEY_VOL_DOWN,
  KEY_VOL_MUTE,
  KEY_VOL_END = 139,

  /*!< Playback Key */
  KEY_PB_BEGIN = 144,
  KEY_PB_PLAY = 144,
  KEY_PB_PAUSE,
  KEY_PB_STOP,
  KEY_PB_PLAYPAUSE,
  KEY_PB_PLAYSTOP,
  KEY_PB_PLAYPAUSESTOP,
  KEY_PB_REPEAT,
  KEY_PB_FIRST,
  KEY_PB_LAST,
  KEY_PB_PREV,
  KEY_PB_NEXT,
  // Add more TBD here if any
  KEY_PB_END = 159,

  /*!< Function Key */
  KEY_FUNC_BEGIN = 160,
  KEY_FUNC_CTRL = 160,
  KEY_FUNC_OPTION,

  KEY_FUNC_PIANOPLAY = 164,
  KEY_FUNC_PIANOLIB,
  KEY_FUNC_PIANOMAGIC,
  // Add more TBD here if any

  KEY_FUNC_GAMENOTECATCH = 176,
  KEY_FUNC_GAMECOMPOSE,
  KEY_FUNC_GAMESUPERVOICE,
  KEY_FUNC_GAMEWEATHERCAST,
  // Add more TBD here if any
  KEY_FUNC_END = 191,

  /*!< Digital/Alphabet Key */
  KEY_DIGIALAPH_BEGIN = 200,
  KEY_NUM_0 = 200,
  KEY_NUM_1,
  KEY_NUM_2,
  KEY_NUM_3,
  KEY_NUM_4,
  KEY_NUM_5,
  KEY_NUM_6,
  KEY_NUM_7,
  KEY_NUM_8,
  KEY_NUM_9,

  KEY_ALPHABET_A,
  KEY_ALPHABET_B,
  KEY_ALPHABET_C,
  KEY_ALPHABET_D,
  KEY_ALPHABET_E,
  KEY_ALPHABET_F,
  KEY_ALPHABET_G,
  KEY_ALPHABET_H,
  KEY_ALPHABET_I,
  KEY_ALPHABET_J,
  KEY_ALPHABET_K,
  KEY_ALPHABET_L,
  KEY_ALPHABET_M,
  KEY_ALPHABET_N,
  KEY_ALPHABET_O,
  KEY_ALPHABET_P,
  KEY_ALPHABET_Q,
  KEY_ALPHABET_R,
  KEY_ALPHABET_S,
  KEY_ALPHABET_T,
  KEY_ALPHABET_U,
  KEY_ALPHABET_V,
  KEY_ALPHABET_W,
  KEY_ALPHABET_X,
  KEY_ALPHABET_Y,
  KEY_ALPHABET_Z,
  KEY_DIGIALAPH_END = 239,

  KEY_SYMBOL_BEGIN = 240,
  KEY_SYMBOL_AT = 240,
  KEY_SYMBOL_ADDR,
  KEY_SYMBOL_STAR,
  KEY_SYMBOL_SHARP,
  KEY_SYMBOL_PLUS,
  KEY_SYMBOL_MINUS,
  KEY_SYMBOL_DOT,
  KEY_SYMBOL_COMMA,
  KEY_SYMBOL_QUOTA,
  KEY_SYMBOL_COLON,
  KEY_SYMBOL_QUESTION,
  KEY_SYMBOL_BANG,
  KEY_SYMBOL_ENTER,
  KEY_SYMBOL_DEL,
  KEY_SYMBOL_END = 254,

  /** Be sure not bigger than 255 */
  KEY_UNDEFINED = 0xFF
};

enum {
  /*!< Standard Piano Key, Start from 21 and end at 108 */
  /**
   * There are 128 possible notes on a MIDI device, numbered 0 to 127
   * (where Middle C is note number 60).
   */
  KEY_PIANO_0 = 0,
                      /* 0~20 Reserved */
  KEY_PIANO_21 = 21 , /*  21 */
  KEY_PIANO_22      , /*  22 */
  KEY_PIANO_23      , /*  23 */
  KEY_PIANO_24      , /*  24 */
  KEY_PIANO_25      , /*  25 */
  KEY_PIANO_26      , /*  26 */
  KEY_PIANO_27      , /*  27 */
  KEY_PIANO_28      , /*  28 */
  KEY_PIANO_29      , /*  29 */
  KEY_PIANO_30      , /*  30 */
  KEY_PIANO_31      , /*  31 */
  KEY_PIANO_32      , /*  32 */
  KEY_PIANO_33      , /*  33 */
  KEY_PIANO_34      , /*  34 */
  KEY_PIANO_35      , /*  35 */
  KEY_PIANO_36      , /*  36 */
  KEY_PIANO_37      , /*  37 */
  KEY_PIANO_38      , /*  38 */
  KEY_PIANO_39      , /*  39 */
  KEY_PIANO_40      , /*  40 */
  KEY_PIANO_41      , /*  41 */
  KEY_PIANO_42      , /*  42 */
  KEY_PIANO_43      , /*  43 */
  KEY_PIANO_44      , /*  44 */
  KEY_PIANO_45      , /*  45 */
  KEY_PIANO_46      , /*  46 */
  KEY_PIANO_47      , /*  47 */
  KEY_PIANO_48      , /*  48 */
  KEY_PIANO_49      , /*  49 */
  KEY_PIANO_50      , /*  50 */
  KEY_PIANO_51      , /*  51 */
  KEY_PIANO_52      , /*  52 */
  KEY_PIANO_53      , /*  53 */
  KEY_PIANO_54      , /*  54 */
  KEY_PIANO_55      , /*  55 */
  KEY_PIANO_56      , /*  56 */
  KEY_PIANO_57      , /*  57 */
  KEY_PIANO_58      , /*  58 */
  KEY_PIANO_59      , /*  59 */
  KEY_PIANO_60      , /*  60 */
  KEY_PIANO_61      , /*  61 */
  KEY_PIANO_62      , /*  62 */
  KEY_PIANO_63      , /*  63 */
  KEY_PIANO_64      , /*  64 */
  KEY_PIANO_65      , /*  65 */
  KEY_PIANO_66      , /*  66 */
  KEY_PIANO_67      , /*  67 */
  KEY_PIANO_68      , /*  68 */
  KEY_PIANO_69      , /*  69 */
  KEY_PIANO_70      , /*  70 */
  KEY_PIANO_71      , /*  71 */
  KEY_PIANO_72      , /*  72 */
  KEY_PIANO_73      , /*  73 */
  KEY_PIANO_74      , /*  74 */
  KEY_PIANO_75      , /*  75 */
  KEY_PIANO_76      , /*  76 */
  KEY_PIANO_77      , /*  77 */
  KEY_PIANO_78      , /*  78 */
  KEY_PIANO_79      , /*  79 */
  KEY_PIANO_80      , /*  80 */
  KEY_PIANO_81      , /*  81 */
  KEY_PIANO_82      , /*  82 */
  KEY_PIANO_83      , /*  83 */
  KEY_PIANO_84      , /*  84 */
  KEY_PIANO_85      , /*  85 */
  KEY_PIANO_86      , /*  86 */
  KEY_PIANO_87      , /*  87 */
  KEY_PIANO_88      , /*  88 */
  KEY_PIANO_89      , /*  89 */
  KEY_PIANO_90      , /*  90 */
  KEY_PIANO_91      , /*  91 */
  KEY_PIANO_92      , /*  92 */
  KEY_PIANO_93      , /*  93 */
  KEY_PIANO_94      , /*  94 */
  KEY_PIANO_95      , /*  95 */
  KEY_PIANO_96      , /*  96 */
  KEY_PIANO_97      , /*  97 */
  KEY_PIANO_98      , /*  98 */
  KEY_PIANO_99      , /*  99 */
  KEY_PIANO_100     , /* 100 */
  KEY_PIANO_101     , /* 101 */
  KEY_PIANO_102     , /* 102 */
  KEY_PIANO_103     , /* 103 */
  KEY_PIANO_104     , /* 104 */
  KEY_PIANO_105     , /* 105 */
  KEY_PIANO_106     , /* 106 */
  KEY_PIANO_107     , /* 107 */
  KEY_PIANO_108     , /* 108 */
};


#define KEY(N)                      KEY_PIANO_##N

#define KEY_FCC_TEST_BEGIN          59
#define KEY_FCC_TEST_END            67

#define KEY_BOARD_BEGIN             KEY(53)
#define KEY_BOARD_END               KEY(76)

#define KEY_BOARD_CNT               (KEY_BOARD_END - KEY_BOARD_BEGIN + 1)

#define KEY_BOARD_CHRT_BEGIN        KEY(53)
#define KEY_BOARD_CHRT_END          KEY(59)

#define KEY_BOARD_CHINV_BEGIN       KEY(60)
#define KEY_BOARD_CHINV_END         KEY(76)

#define KEY_MID_C                   KEY(60)

#define KEY_OFFSET_PER_DEGREE       12

#define KEY_BOARD_MID_C             KEY_PIANO_L1_C

#define KEY_SIMU_MAX                4   // How many keys we can support simultaneously
#define KEY_FUN_MAX                 3   // Play an interesting voice for fun if several key press simultaneously
#define KEY_IGNORE_MAX              2   // If press function key or nv key more than two key then ignore the event

enum {
  KEY_ACTION_PRESSED,
  KEY_ACTION_LONGPRESS,
  KEY_ACTION_RELEASED,
  KEY_ACTION_DOUBLE_CLICK,
};


#endif /* __KEY_H__ */

/* vim: set ts=2 sw=2 tw=0 list : */
