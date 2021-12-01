/* ===================================================================== *
 *                                                                       *
 * REMOTE COMMANDS VIA SERIAL
 *                                                                       *
 * ===================================================================== */
 
 #define BUF_LENGTH 128  /* Buffer for the incoming command. */

static bool do_echo = true;

void doRemoteCommands() {
    /* Process incoming commands. */
    while (Serial.available()) {
        static char buffer[BUF_LENGTH];
        static int length = 0;

        int data = Serial.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) Serial.write("\b \b");
            }
        }
        else if (data == '\r') {
            if (do_echo) Serial.write("\r\n");    // output CRLF
            buffer[length] = '\0';
            if (length) exec(buffer);
            length = 0;
        }
        else if (length < BUF_LENGTH - 1) {
            buffer[length++] = data;
            if (do_echo) Serial.write(data);
        }
    }
}

/* Execute a complete command. */
static void exec(char *cmdline)
{
    char *command = strsep(&cmdline, " ");

    if (strcmp_P(command, PSTR("help")) == 0) {
        Serial.println(F(
            "mode <pin> <mode>\r\n"
            "read <pin>\r\n"
            "write <pin> <value>\r\n"
            "sethour <hh>\r\n"
            "setmin <mm>\r\n"
            "poolon\r\n"
            "solaron\r\n"));
    } else if (strcmp_P(command, PSTR("read")) == 0) {
        int pin = atoi(cmdline);
        Serial.println(digitalRead(pin));
    } else if (strcmp_P(command, PSTR("write")) == 0) {
        int pin = atoi(strsep(&cmdline, " "));
        int value = atoi(cmdline);
        digitalWrite(pin, value);
    } else if (strcmp_P(command, PSTR("getstate")) == 0) {
      printJsonState();
    } else if (strcmp_P(command, PSTR("poolon")) == 0) {
      g_poolOnOverride = true;
    } else if (strcmp_P(command, PSTR("solaron")) == 0) {
      g_solarOnOverride = true;
    } else if (strcmp_P(command, PSTR("pooloff")) == 0) {
      g_poolOnOverride = false;
    } else if (strcmp_P(command, PSTR("solaroff")) == 0) {
      g_solarOnOverride = false;
    } else if (strcmp_P(command, PSTR("save")) == 0) {
      saveDefaults();
    } else if (strcmp_P(command, PSTR("set")) == 0) {
        g_dirty = true;
        char *setcommand = strsep(&cmdline, " ");
        if (strcmp_P(setcommand, PSTR("hour")) == 0) {
          int h = atoi(strsep(&cmdline, " "));
          g_clock_hour = h;
        } else if (strcmp_P(setcommand, PSTR("minute")) == 0) {
          int m = atoi(strsep(&cmdline, " "));
          g_clock_minute = m;
        } else if (strcmp_P(setcommand, PSTR("clock")) == 0) {
          int h = atoi(strsep(&cmdline, " "));
          int m = atoi(strsep(&cmdline, " "));
          g_clock_hour = h;
          g_clock_minute = m;
        } else if (strcmp_P(setcommand, PSTR("pool")) == 0) {
          char *setpoolcommand = strsep(&cmdline, " ");
          if (strcmp_P(setpoolcommand, PSTR("on")) == 0) {
            int h = atoi(strsep(&cmdline, " "));
            int m = atoi(strsep(&cmdline, " "));
            g_poolOnHour = h;
            g_poolOnMinute = m;
          } else if (strcmp_P(setpoolcommand, PSTR("off")) == 0) {
            int h = atoi(strsep(&cmdline, " "));
            int m = atoi(strsep(&cmdline, " "));
            g_poolOffHour = h;
            g_poolOffMinute = m;
          } else {
            #ifdef DEBUG
              Serial.print(F("DEBUG: set pool UNKNOWN COMMAND: "));
              Serial.println(setcommand);
            #endif
          }
        } else if (strcmp_P(setcommand, PSTR("solar")) == 0) {
          char *setsolarcommand = strsep(&cmdline, " ");
          if (strcmp_P(setsolarcommand, PSTR("on")) == 0) {
            int h = atoi(strsep(&cmdline, " "));
            int m = atoi(strsep(&cmdline, " "));
            g_solarOnHour = h;
            g_solarOnMinute = m;
          } else if (strcmp_P(setsolarcommand, PSTR("off")) == 0) {
            int h = atoi(strsep(&cmdline, " "));
            int m = atoi(strsep(&cmdline, " "));
            g_solarOffHour = h;
            g_solarOffMinute = m;
          } else if (strcmp_P(setsolarcommand, PSTR("min")) == 0) {
            int c = atoi(strsep(&cmdline, " "));
            g_solarOnTemp = c;
          } else if (strcmp_P(setsolarcommand, PSTR("max")) == 0) {
            int c = atoi(strsep(&cmdline, " "));
            g_solarOffTemp = c;
          } else {
            #ifdef DEBUG
              Serial.print(F("DEBUG: set solar UNKNOWN COMMAND: "));
              Serial.println(setcommand);
            #endif
          }
        } else {
          // No set command known
          #ifdef DEBUG
            Serial.print(F("DEBUG: set UNKNOWN COMMAND: "));
            Serial.println(setcommand);
          #endif
        }
    }
     else {
        Serial.print(F("ERROR: Unknown command: "));
        Serial.println(command);
    }
}
