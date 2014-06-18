/*
* This file is part of LEGO-Pi.
*
* Copyright (Copyplease) szz-dvl.
*
*
* License
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Affero General Public License for more details at
* <http://www.gnu.org/licenses/agpl-3.0-standalone.html>
*/

#include <lego/lego_digital.h>
#include <getopt.h>

int def_port = 0;

static int tst = 0;
static int port = 0;
static bool log_dbg = false;
static bool titled = true;
static int  tdepend = true;

static void print_usage(const char *prog)
{
	printf("Usage: %s -t<test_num> [-peTd]\n", prog);
	puts("  -t --test      test number to perform [1-6]             {mandatory}\n"
	     "  -p --port      digital port [0-1]                       {0}\n"
	     "  -e --tdep      extra paramter, test dependant           {1}\n"
	     "  -T --titled    Print titles before info fields [no arg] {true}\n"
	     "  -d --dbg       Set the library in debug mode [no arg]   {false}\n");
	exit(EXIT_FAILURE);
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "test",    1, 0, 't' },
      { "tdep",    1, 0, 'e' },
      { "port",    1, 0, 'p' },
      { "dbg" ,    0, 0, 'd' },
      { "titled",  0, 0, 'T' },
      {0, 0, 0, 0},
    };
      
    int c;
    
    c = getopt_long(argc, argv, "t:e:p:dT", lopts, NULL);
      
    if (c == -1)
      break;
    
    switch (c) {
    case 't':
      tst = atoi(optarg);
      break;
    case 'e':
      tdepend = atoi(optarg);
      break;
    case 'p':
      port = atoi(optarg);
      break;
    case 'd':
      log_dbg = true;
      break;
    case 'T':
      titled = true;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}

//usefull for testing
int get_in(char *to_print, int type){
  switch(type){
  case 1:
    {
      int ret_int;
      printf("%s", to_print);
      scanf("\n%d",&ret_int);
      fflush(NULL);
      return ret_int;
    }
    break;;
  default:
    {
      char ret;
      printf("%s", to_print);
      scanf("\n%c",&ret);
      fflush(NULL);
      return ret;
    }
    break;;
  }
}

int main (int argc, char * argv[]) {

  parse_opts(argc, argv);
  
  dg_init(7);
  dg_set_verbose(log_dbg ? LOG_LVL_DBG : LOG_LVL_ADV);
  
  switch (tst) {
  case 0:
    printf("At least the test number is needed!\n");
    dg_shutdown();
    print_usage(argv[0]);
  case 1://HT_COLOR Test
    { 
      bool cal = tdepend, ret;
      uint8_t red, green, blue, cnum, cidx, state;
      uint16_t redraw, greenraw, blueraw, white;
      int k;
      char * info [DG_INFO_TABLE_TAM];

      DGDVC col;
      if((ret = dg_new(&col, HT_COLOR, port)))
	printf("New device successfully created: type >> %d, version >> %d, port >> %d\n", col.type, col.vers, col.port);
      else
	printf("Error creating device\n");

      if(ret) {
	
	if(!dg_get_state(&col, &state))
	  printf("Error getting initial state\n");
	else
	  printf("Initial state: %u\n", state);

	if(!dg_get_info(&col, info, titled))
	  printf("Error getting sensor info.\n");
	else {
	  for (k=0; strcmp(info[k], "") != 0; k++)
	    printf("%s\n", info[k]);

	}
	printf("\n");
	

	if(dg_col_get_number(&col, &cnum))
	  printf("Col. number: %u\n", cnum);
	else
	  printf("Error getting color number\n");
	
	if(dg_col_get_index(&col, &cidx)) {
	  printf("Col. index: ");
	  for (k=BYTE_LEN-3; k >= 0; k--) {
	    if (cidx & (0x01 << k))
	      printf("1");
	    else 
	      printf("0");
	  }
	  printf(", 0x%02x\n", cidx);
	} else
	  printf("Error getting color index\n");
	
	if(dg_col_get_rgb(&col, &red, &green, &blue))
	  printf("Red: %u, Green: %u, Blue: %u\n", red, green, blue);
	else
	  printf("Error getting RGB\n");
	
	if(dg_col_get_norm(&col, &red, &green, &blue))
	  printf("Red_nrm: %u, Green_nrm: %u, Blue_nrm: %u\n", red, green, blue);
	else
	  printf("Error getting normalised values\n");
	
	if(dg_col_get_raw(&col, &redraw, &greenraw, &blueraw, false))
	  printf("Red_raw: %u, Green_raw: %u, Blue_raw: %u\n", redraw, greenraw, blueraw);
	else
	  printf("Error getting raw values\n");
	
	if(col.vers == 2) {
	  printf("\nTrying to get white channel...\n");
	  
	  if(dg_col_get_white(&col, &white, false, false))
	    printf("White_channel: %u \n", white);
	  else
	    printf("Error getting white channel\n");
	} else if(cal) {
	  
	  printf("\nTrying to callibrate white ...\n");
	  
	  dg_send_cmd(&col, HTCS_CAL_WHITE);
	  
	}

	if(!dg_get_state(&col, &state))
	  printf("Error getting final state\n");
	else
	  printf("Final state: %u\n", state);
	
      }
      
    }
    break;;
  case 2: //LEGO_UltraSonic Test
    {
      
      DGDVC us;
      bool ret;
      uint8_t state, dist, tdist[8];
      int i;
      char * info [DG_INFO_TABLE_TAM];
      
      if((ret = dg_new(&us, LEGO_US, port)))
	printf("New device successfully created: type >> %d, version >> %d, port >> %d\n", us.type, us.vers, us.port);
      else
	printf("Error creating device\n");
      
      if(ret){
	
	if(!dg_get_state(&us, &state))
	  printf("Error getting initial state\n");
	else
	  printf("Initial state: %u\n", state);
	
	if(!dg_get_info(&us, info, titled))
	  printf("Error getting sensor info.\n");
	else {
	  for (i=0; strcmp(info[i],  "") != 0; i++)
	    printf("%s\n", info[i]);

	}
	
	printf("\nAttempting to get all distances one by one\n");

	for(i=0; i<8; i++) {
	  if(!dg_us_get_dist(&us, &dist, i))
	    printf("Error getting distance: %d\n", i);
	  else
	    printf("Distance %d: %u\n", i, dist);

	}

	printf("\nAttempting to get all distances at once\n");

	if(!dg_us_get_alldist(&us, tdist))
	  printf("Error getting all distances.\n");
	else {
	  for(i=0; i<8; i++)
	    printf("Distance %d: %u\n", i, tdist[i]);
	}

	printf("\n");
	if(!dg_get_state(&us, &state))
	  printf("Error getting final state\n");
	else
	  printf("Final state: %u\n", state);
	
      }
      

    }
    break;;
  case 3: //HT_IRSeeker Test
    {
      
      bool dc = tdepend;
      bool ret;
      DGDVC irs;
      uint8_t dir, str, avg, state;
      uint8_t table[5];
      int i;
      
      char * info [DG_INFO_TABLE_TAM];
      
      if((ret = dg_new(&irs, HT_IRS, port)))
	printf("New device successfully created: type >> %d, version >> %d, port >> %d\n\n", irs.type, irs.vers, irs.port);
      else
	printf("Error creating device\n");

      if(ret){
	
	if(!dg_get_state(&irs, &state))
	  printf("Error getting initial state\n");
	else
	  printf("Initial state: %u\n", state);
	
	
	if(!dg_get_info(&irs, info, titled))
	  printf("Error getting sensor info.\n");
	else {
	  for (i=0; strcmp(info[i],  "") != 0; i++)
	    printf("%s\n", info[i]);
	}
	
	printf("\n");

	if(!dg_irs_get_dir(&irs, &dir, dc))
	  printf("Error getting %s direction\n", dc ? "DC" : "AC");
	else 
	  printf("%s direction = %u\n", dc ? "DC" : "AC", dir);
	
	printf("\nGetting stregths one by one\n");

	for (i = 1; i<=5; i++) {
	  if(!dg_irs_get_str(&irs, &str, i, dc))
	    printf("Error getting %s direction %d\n", dc ? "DC" : "AC", i);
	  else 
	    printf("%s stregth %d = %u\n", dc ? "DC" : "AC", i, str);
	}
	
	printf("\nGetting all stregths at once\n");

	if(!dg_irs_get_allstr(&irs, table, dc))
	  printf("Error getting stregths table\n");
	else {
	  for (i = 0; i<5; i++)
	    printf("%s stregth %d = %u\n", dc ? "DC" : "AC", i+1, table[i]);
	}
	
	if(irs.vers == 1)
	  printf("\nAttempting to get DC average on version 1...\n\n");

	if(!dg_irs_get_dcavg(&irs, &avg))
	  printf("Error getting DC average%s.\n", irs.vers == 1 ? ", as expected" : "");
	else
	  printf("DC average is: %u\n", avg); //Fuck me...
	
	if(!dg_get_state(&irs, &state))
	  printf("Error getting final state\n");
	else
	  printf("Final state: %u\n", state);

      }

    }
    break;;
  case 4: //HT_ACCELerometer Test
    {
      DGDVC acc;
      bool ret;
      uint8_t state;
      int i, x, y, z;
      char * info [DG_INFO_TABLE_TAM];
      
      if((ret = dg_new(&acc, HT_ACCEL, port)))
	printf("New device successfully created: type >> %d, version >> %d, port >> %d\n", acc.type, acc.vers, acc.port);
      else
	printf("Error creating device\n");
      
      if(ret){
	
	if(!dg_get_state(&acc, &state))
	  printf("Error getting initial state\n");
	else
	  printf("Initial state: %u\n", state);
	
	if(!dg_get_info(&acc, info, titled))
	  printf("Error getting sensor info.\n");
	else {
	  for (i=0; strcmp(info[i],  "") != 0; i++)
	    printf("%s\n", info[i]);
	}
	
	printf("\nAttempting to get axis:\n");

	if(!dg_acc_get_axis(&acc, &x, &y, &z))
	    printf("Error getting axis\n");
	  else
	    printf("X: %d\nY: %d\nZ: %d\n", x, y, z);

       	printf("\n");
	if(!dg_get_state(&acc, &state))
	  printf("Error getting final state\n");
	else
	  printf("Final state: %u\n", state);
	
      }

    }
    break;;
  case 5: //HT_COMPASS Test
    {
      DGDVC com;
      bool ret;
      uint8_t state;
      uint16_t tdhead, wrhead;
      int i;
      char * info [DG_INFO_TABLE_TAM];

      if((ret = dg_new(&com, HT_COMPASS, port)))
	  printf("New device successfully created: type >> %d, version >> %d, port >> %d\n", com.type, com.vers, com.port);
	else
	  printf("Error creating device\n");

      if(tdepend && ret) {  //calibration
	
	if(!dg_send_cmd(&com, HTCM_CAL_MD))
	  printf("Error setting calibration mode\n");

	get_in("Calibration finished? ",1); //wait for approximately a turn and a half.

	if(!dg_send_cmd(&com, HTCM_MES_MD))
	  printf("Error setting measurement mode\n");
	
	if(!dg_get_state(&com, &state)) //strange behavior here, needs at least 5 retries to get the value properly ...[15-6-2014]
	  printf("Error getting calibration result\n");
	
	if(!dg_com_get_head(&com, &tdhead, &wrhead))
	  printf("Error getting axis\n");
	else
	  printf("2DG_head: %u\nWDG_head: %u\n", tdhead, wrhead);
	
	printf("Calibration %s\n", state == 2 ? "FAIL" : "SUCCESSFULL");

      } else if (ret) {
	  
	  if(!dg_get_state(&com, &state))
	    printf("Error getting initial state\n");
	  else
	    printf("Initial state: %u\n", state);
	  
	  if(!dg_get_info(&com, info, titled))
	    printf("Error getting sensor info.\n");
	  else {
	    for (i=0; strcmp(info[i],  "") != 0; i++)
	      printf("%s\n", info[i]);
	  }
	  
	  printf("\nAttempting to get heading:\n");

	  if(!dg_com_get_head(&com, &tdhead, &wrhead))
	    printf("Error getting axis\n");
	  else
	    printf("2DG_head: %u\nWDG_head: %u\n", tdhead, wrhead);
	  
	  printf("\n");
	  if(!dg_get_state(&com, &state))
	    printf("Error getting final state\n");
	  else
	    printf("Final state: %u\n", state);
      }

    }
    break;;
  case 6: //DG_OTHER Test
    {
      int port0 = port;
      int port1 = tdepend;
      DGDVC p0, p1;
      bool ret0, ret1;
      uint8_t data_out0[] = {0x10};
      uint8_t data_out1[] = {0x42};
      uint16_t data_in0[6], data_in1[1]; 
      int i;
      
      if((ret0 = dg_new_unknown(&p0, LEGO_ADDR, LEGO_FREQ, port0)))
	printf("Device plugged to port %d successfully created\n", port0);
      else
	printf("Failed to create device on port %d\n", port0);
      
      if((ret1 = dg_new_unknown(&p1, LEGO_ADDR, LEGO_FREQ, port1)))
	printf("Device plugged to port %d successfully created\n", port1);
      else
	printf("Failed to create device on port %d\n", port1);

      if(ret0) {
	printf("Attempting to get sensor type on port %d [gessing LEGO_US ...]\n", port0);

	if(dg_transfer(&p0, data_out0, 1, true, data_in0, 6, false, 5000)){
	  printf("Sensor Type: ");
	  for(i=0; i<6; i++)
	    printf("%c", (char)data_in0[i]);

	  printf("\n");
	  
	} else 
	  printf("Failed reading sensor type on port %d\n", port0);

      }

      if(ret1) {

	printf("Attempting to get color number on port %d [gessing HT_COLOR ...]\n", port1);

	if(!dg_write(&p1, data_out1, 1, 0))
	  printf("Failed to wtite data to device on port %d", port1);
	else {
	  if(dg_read(&p1, data_in1, 1, false, 0))
	    printf("Color number: %u\n", data_in1[0]);
	  else 
	    printf("Failed reading color number on port %d\n", port1);
	  
	}
	
      }
      
    }
    break;;
  default:
    break;;
  }
  dg_shutdown();
  return (EXIT_SUCCESS);
}
