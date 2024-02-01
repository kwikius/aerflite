// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

#if CLI_ENABLED == ENABLED

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] = {
    // command			function called
    // =======          ===============
    {"reset",           MENU_FUNC(setup_factory)},
    {"erase",           MENU_FUNC(setup_erase)},
    {"show",            MENU_FUNC(setup_show)},
    {"set",             MENU_FUNC(setup_set)}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
int8_t Plane::setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf("Setup Mode\n"
                         "\n"
                         "IMPORTANT: if you have not previously set this system up, use the\n"
                         "'reset' command to initialize the EEPROM to sensible default values\n"
                         "and then the 'radio' command to configure for your radio.\n"
                         "\n");

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
int8_t Plane::setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int c;

    cliSerial->printf("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: ");

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    AP_Param::erase_all();
    cliSerial->printf("\nFACTORY RESET complete - please reset board to continue");

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
int8_t Plane::setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf("Invalid command. Usage: set <name> <value>\n");
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf("Param not found: %s\n", argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf("Value out of range for type INT8\n");
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf("Value out of range for type INT16\n");
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf("Cannot set parameter of type %d.\n", p_type);
            break;
    }

    return 0;
}

void Plane::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}


int8_t Plane::setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf("Parameter not found: '%s'\n", argv[1].str);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    //report_version();
   // report_radio();
   // report_frame();
   // report_batt_monitor();
   // report_flight_modes();
   // report_ins();
   // report_compass();
   // report_optflow();

    AP_Param::show_all(cliSerial);

    return(0);
}


int8_t Plane::setup_erase(uint8_t argc, const Menu::arg *argv)
{
    int c;

    cliSerial->printf("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: ");

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    zero_eeprom();
    return 0;
}

void Plane::zero_eeprom(void)
{
    cliSerial->printf("\nErasing EEPROM\n");
    StorageManager::erase();
    cliSerial->printf("done\n");
}

#endif // CLI_ENABLED
