/* Original source is AM2321.c by Masayuki Takagi
 * convert to AM2320 & WBGT add by pochi_ken
 * presented by Nikkei Linux 2020/09
 */
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>              /* for O_RDWR */
#include <string.h>             /* for memcpy */
#include <linux/i2c-dev.h>      /* for I2C_SLAVE */

/* I2C character device */
#define I2C_DEVICE "/dev/i2c-1"

/* I2C address of AM2320 sensor in 7 bits
 * - the first 7 bits should be passed to ioctl system call
 *   because the least 1 bit of the address represents read/write
 *   and the i2c driver takes care of it
 */
#define AM2320_ADDR (0xB8 >> 1)


/*
 *  udelay function
 */
long timeval_to_usec( struct timeval tm ) {
  return tm.tv_sec * 1000000 + tm.tv_usec;
}

void udelay( long us ) {
  struct timeval current;
  struct timeval start;

  gettimeofday( &start, NULL );
  do {
    gettimeofday( &current, NULL );
  } while( timeval_to_usec( current ) - timeval_to_usec( start ) < us );
}


/*
 *  CRC16
 */
unsigned short crc16( unsigned char *ptr, unsigned char len ) {
  unsigned short crc = 0xFFFF;
  unsigned char i;

  while( len-- )
  {
    crc ^= *ptr++;
    for( i = 0; i < 8; i++ ) {
      if( crc & 0x01 ) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

unsigned char crc16_low( unsigned short crc ) {
  return crc & 0xFF;
}

unsigned char crc16_high( unsigned short crc ) {
  return crc >> 8;
}


/*
 *  st_am2320 Structure
 */

typedef struct {
  unsigned char data[8];
} st_am2320;

void __check_crc16( st_am2320 measured ) {
  unsigned short crc_m, crc_s;

  crc_m = crc16( measured.data, 6 );
  crc_s = (measured.data[7] << 8) + measured.data[6];
  if ( crc_m != crc_s ) {
    fprintf( stderr, "am2320: CRC16 does not match\n" );
    exit( 1 );
  }

  return;
}

st_am2320 __st_am2320( unsigned char* data ) {
  st_am2320 result;
  memcpy( result.data, data, 8 );
  __check_crc16( result );
  return result;
}

void am2320_dump( st_am2320 measured ) {
  printf( "[ 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ]\n",
          measured.data[0], measured.data[1],
          measured.data[2], measured.data[3],
          measured.data[4], measured.data[5],
          measured.data[6], measured.data[7] );
  return;
}

short __am2320_temperature( st_am2320 measured ) {
  return (measured.data[4] << 8) + measured.data[5];
}

short am2320_temperature_integral( st_am2320 measured ) {
  return __am2320_temperature( measured ) / 10;
}

short am2320_temperature_fraction( st_am2320 measured ) {
  return __am2320_temperature( measured ) % 10;
}

short __am2320_humidity( st_am2320 measured ) {
  return (measured.data[2] << 8) + measured.data[3];
}

short am2320_humidity_integral( st_am2320 measured ) {
  return __am2320_humidity( measured ) / 10;
}

short am2320_humidity_fraction( st_am2320 measured ) {
  return __am2320_humidity( measured ) % 10;
}


/*
 *  Measurement function
 */

st_am2320 am2320() {
  int fd;
  int ret;
  int retry_cnt;
  unsigned char data[8];

  /* open I2C device */
  fd = open( I2C_DEVICE, O_RDWR );
  if ( fd < 0 ) {
    perror( "am2320(1)" );
    exit( 1 );
  }

  /* set address of I2C device in 7 bits */
  ret = ioctl( fd, I2C_SLAVE, AM2320_ADDR );
  if ( ret < 0 ) {
    perror( "am2320(2)" );
    exit( 2 );
  }

 retry_cnt = 0;
 retry:

  /* wake I2C device up */
  write( fd, NULL, 0);

  /* write measurement request */
  data[0] = 0x03; data[1] = 0x00; data[2] = 0x04;
  ret = write( fd, data, 3 );
  if ( ret < 0 && retry_cnt++ < 5 ) {
    fprintf( stderr, "am2320: retry\n" );
    udelay( 1000 );
    goto retry;
  }
  if ( ret < 0 ) {
    perror( "am2320(3)" );
    exit( 3 );
  }

  /* wait for having measured */
  udelay( 1500 );

  /* read measured result */
  memset( data, 0x00, 8 );
  ret = read( fd, data, 8 );
  if ( ret < 0 ) {
    perror( "am2320(4)" );
    exit( 4 );
  }

  /* close I2C device */
  close( fd );

  return __st_am2320( data );
}

st_am2320 am2320_stub() {
  unsigned char data[] = { 0x03, 0x04, 0x02, 0x0B,
                           0x01, 0x13, 0xcC, 0xCF };
  return __st_am2320( data );
}


/*
 *  Print functions
 */

void print_am2320( st_am2320 measured ) {
  printf( "%d.%d %d.%d\n",
          am2320_temperature_integral( measured ),
          am2320_temperature_fraction( measured ),
          am2320_humidity_integral( measured ),
          am2320_humidity_fraction( measured ) );
  return;
}

void print_am2320_human_readable( st_am2320 measured ) {
  printf( "Temperature %d.%d [C]\n",
          am2320_temperature_integral( measured ),
          am2320_temperature_fraction( measured ) );
  printf( "Humidity    %d.%d [%%]\n",
          am2320_humidity_integral( measured ),
          am2320_humidity_fraction( measured ) );
  return;
}

/*
 * WBGT calc
 */
void print_am2320_wbgt( st_am2320 measured ) {
  int int_t, wbgt_t;
  int int_h, wbgt_h;
  char *wbgt_result;
  unsigned char wbgt[22][17] = {{ 15, 15, 16, 16, 17, 17, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24 },
                                { 15, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25 },
                                { 16, 17, 17, 18, 19, 19, 20, 20, 21, 22, 22, 23, 23, 24, 25, 25, 26 },
                                { 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 26, 26, 27 },
                                { 18, 18, 19, 20, 20, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28 },
                                { 18, 19, 20, 20, 21, 22, 22, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29 },
                                { 19, 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30 },
                                { 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 27, 28, 28, 29, 30, 30, 31 },
                                { 21, 21, 22, 23, 24, 24, 25, 26, 26, 27, 28, 29, 29, 30, 31, 31, 32 },
                                { 21, 22, 23, 24, 24, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33 },
                                { 22, 23, 24, 24, 25, 26, 27, 27, 28, 29, 30, 30, 31, 32, 33, 33, 34 },
                                { 23, 24, 25, 25, 26, 27, 28, 28, 29, 30, 31, 31, 32, 33, 34, 34, 35 },
                                { 24, 25, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36 },
                                { 25, 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 33, 34, 35, 36, 37, 37 },
                                { 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 33, 34, 35, 36, 37, 38 ,38 },
                                { 26, 27, 28, 29, 29, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 39 },
                                { 27, 28, 29, 29, 30, 31, 32, 33, 35, 35, 35, 36, 37, 38, 39, 40, 41 },
                                { 28, 28, 29, 30, 31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42 },
                                { 28, 29, 30, 31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43 },
                                { 29, 30 ,31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 },
                                { 30, 31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 },
                                { 31, 32, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46 }};
  int_t = am2320_temperature_integral( measured ) + ( am2320_temperature_fraction( measured ) + 9 ) / 10;
  if ( int_t >= 43 ) {
    wbgt_t = 21;
  } else if ( int_t < 21 ) {
    wbgt_t = 0;
  } else {
    wbgt_t = int_t - 21;
  }
  int_h = am2320_humidity_integral( measured ) + ( am2320_humidity_fraction( measured ) + 9 ) / 10;
  if ( int_h < 20 ) {
    wbgt_h = 0;
  } else if ( int_h >= 100 ) {
    wbgt_h = 16;
  } else {
    wbgt_h = ( int_h - 20 ) / 5 + 1;
  }
  if ( wbgt[ wbgt_t ][ wbgt_h ] >= 31 ) {
    wbgt_result = "Danger";
  } else if ( wbgt[ wbgt_t ][ wbgt_h ] < 31 && wbgt[ wbgt_t ][ wbgt_h ] >= 28 ) {
    wbgt_result = "Strict caution";
  } else if ( wbgt[ wbgt_t ][ wbgt_h ] < 28 && wbgt[ wbgt_t ][ wbgt_h ] >= 25 ) {
    wbgt_result = "Warning";
  } else if ( wbgt[ wbgt_t ][ wbgt_h ] < 25 && wbgt[ wbgt_t ][ wbgt_h ] >= 21 ) {
    wbgt_result = "Caution";
  } else {
    wbgt_result = "Safe";
  }
  printf( "WBGT index    %d [C] %s\n", wbgt[ wbgt_t ][ wbgt_h ], wbgt_result );
  return;
}

/*
 *  Main
 */

#define OPT_HUMAN_READABLE 0x1
#define OPT_STUB 0x2

int print_help() {
  fprintf( stderr,
           "Usage: am2320 [-r] [-h]\n"
           "Get temperature and humidity measured with Aosong's AM2320 sensor.\n"
           "  -r    human readable output\n"
           "  -h    Display help message and exit\n");
  exit( 1 );
}

int parse_options( int argc, char* argv[]) {
  int options = 0;
  int flags = 0;

  while( 1+flags < argc && argv[1+flags][0] == '-' ) {
    switch( argv[1+flags][1] ) {
    case 'r': options |= OPT_HUMAN_READABLE; break;
    case 'd': options |= OPT_STUB; break;
    case 'h': print_help(); break;
    default:
      fprintf( stderr, "am2320: Unsupported option \"%s\"\n", argv[1+flags] );
      print_help();
      exit( 1 );
    }
    flags++;
  }

  return options;
}

int is_opt_human_readable( int options ) {
  return options & OPT_HUMAN_READABLE;
}

int is_opt_stub( int options ) {
  return options & OPT_STUB;
}

int main( int argc, char* argv[] ) {
  int options;
  st_am2320 measured;

  /* parse the given options */
  options = parse_options( argc, argv );

  /* measure temperature and humidity */
  /* measured = ! is_opt_stub( options ) ? am2320() : am2320_stub(); */
  measured = am2320();

  /* print measured temperature and humidity */
  if ( is_opt_stub( options ) ) {
    am2320_dump( measured );
  } else if ( ! is_opt_human_readable( options ) ) {
    print_am2320( measured );
  } else {
    print_am2320_human_readable( measured );
    print_am2320_wbgt( measured );
  }

  return 0;
}
