//
// Created by azurec on 2020/11/13.
//

#ifndef ROBOT_ROBOT_H
#define ROBOT_ROBOT_H

#include <iostream>
#include <errno.h>
#include "signal.h"
#include "stdio.h"
#include "sys/resource.h"
#include "sys/time.h"
#include "sys/types.h"
#include "unistd.h"
#include "sys/mman.h"
#include <pthread.h>

// EtherCAT header
#include "ecrt.h"

// CSV header
#include "string.h"
#include <vector>
#include <fstream>
#include <sstream>

#define TASK_FREQUENCY 1000 // Hz
#define TARGET_VELOCITY 100 // PUU (Synapticon default PUU is rpm)
#define OP 8 // operation mode for 0x6060:0  (csv mode)

/* NOTICE:
 * Change TASK_FREQUENCY to higher value (300Hz++) may cause losing heartbeats
 */

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

#define ETHERCAT_STATUS_OP 0x08
#define STATUS_SERVO_ENABLE_BIT (0x04)


// etherCAT object
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainTx = NULL;
static ec_domain_state_t domainTx_state = {};
static ec_domain_t *domainRx = NULL;
static ec_domain_state_t domainRx_state = {};

//==================== PDO =========================
static uint8_t *domainRx_pd = NULL;
static uint8_t *domainTx_pd = NULL;

// length change with number of slave(s)
static ec_slave_config_t *sc[2];
static ec_slave_config_state_t sc_state[2] = {};


// hardware specification
#define SynapticonSlave1 0,0 // 1st slave
#define SynapticonSlave2 0,1 // 2ed slave

#define Synapticon  0x000022D2,0x00000201// vendor id + product id

// state machine enum
typedef enum _workingStatus {
    sys_working_POWER_ON,
    sys_working_SAFE_MODE,
    sys_working_OP_MODE,
    sys_working_LINK_DOWN,
    sys_working_IDLE_STATUS
} workingStatus;

typedef struct _GsysRunningParm {
    workingStatus m_gWorkStatus;
} GsysRunningParm;

GsysRunningParm  gSysRunning;

int ecstate = 0;
int run = 1;

// Offsets for PDO entries
static struct
{
    /* RxPDOs: 0x1600 */
    unsigned int operation_mode[2];
    unsigned int ctrl_word[2];
    unsigned int target_velocity[2];
    unsigned int target_position[2];
    unsigned int target_torque[2];
    /* RxPDOs: 0x1601 */
    unsigned int digital_out1[2];
    unsigned int digital_out2[2];
    unsigned int digital_out3[2];
    unsigned int digital_out4[2];

    /* TxPDOs: 0x1A00 */
    unsigned int status_word[2];
    unsigned int modes_of_operation_display[2];
    unsigned int actual_position[2];
    unsigned int actual_velocity[2];
    unsigned int actual_torque[2];
    /* TxPDOs: 0x1A01 */
    unsigned int second_position[2];
    unsigned int second_velocity[2];
    unsigned int analog_in1[2];
    unsigned int analog_in2[2];
    unsigned int analog_in3[2];
    unsigned int analog_in4[2];
    /* TxPDOs: 0x1A02 */
    unsigned int digital_in1[2];
    unsigned int digital_in2[2];
    unsigned int digital_in3[2];
    unsigned int digital_in4[2];
}offset;


/* NOTICE:
 * domain_reg must aligned to pdo_entries
 * the PDOs are configured to slaves not master
 *  slaves -> master = TxPOD
 *  master -> slaves = RxPOD
 */

// output domain register (RxPDO mapped objects)
ec_pdo_entry_reg_t domain_Rx_reg[] = {
        // slave - 1
        {SynapticonSlave1, Synapticon, 0x6040, 0, &offset.ctrl_word[0]},
        {SynapticonSlave1, Synapticon, 0x6060, 0, &offset.operation_mode[0]},
        {SynapticonSlave1, Synapticon, 0x60FF, 0, &offset.target_velocity[0]},
        {SynapticonSlave1, Synapticon, 0x607A, 0, &offset.target_position[0]},
        {SynapticonSlave1, Synapticon, 0x6071, 0, &offset.target_torque[0]},
        // slave - 2
        {SynapticonSlave2, Synapticon, 0x6040, 0, &offset.ctrl_word[1]},
        {SynapticonSlave2, Synapticon, 0x6060, 0, &offset.operation_mode[1]},
        {SynapticonSlave2, Synapticon, 0x60FF, 0, &offset.target_velocity[1]},
        {SynapticonSlave2, Synapticon, 0x607A, 0, &offset.target_position[1]},
        {SynapticonSlave2, Synapticon, 0x6071, 0, &offset.target_torque[1]},
        {}
};

// input domain register (TxPDO mapped objects)
ec_pdo_entry_reg_t domain_Tx_reg[] = {
        // slave - 1
        {SynapticonSlave1, Synapticon, 0x6041, 0, &offset.status_word[0]},
//        {SynapticonSlave1, Synapticon, 0x6061, 0, &offset.modes_of_operation_display},
//        {SynapticonSlave1, Synapticon, 0x6077, 0, &offset.actual_torque},
        {SynapticonSlave1, Synapticon, 0x6064, 0, &offset.actual_position[0]},
        {SynapticonSlave1, Synapticon, 0x606C, 0, &offset.actual_velocity[0]},

        // slave - 2
        {SynapticonSlave2, Synapticon, 0x6041, 0, &offset.status_word[1]},
//        {SynapticonSlave2, Synapticon, 0x6061, 0, &offset.modes_of_operation_display},
//        {SynapticonSlave2, Synapticon, 0x6077, 0, &offset.actual_torque},
        {SynapticonSlave2, Synapticon, 0x6064, 0, &offset.actual_position[1]},
        {SynapticonSlave2, Synapticon, 0x606C, 0, &offset.actual_velocity[1]},
//        {SynapticonSlave1, Synapticon, 0x230A, 0, &offset.second_position},
//        {SynapticonSlave1, Synapticon, 0x230B, 0, &offset.second_velocity},
//        {SynapticonSlave1, Synapticon, 0x2401, 0, &offset.analog_in1[0]},
//        {SynapticonSlave2, Synapticon, 0x2401, 0, &offset.analog_in1[0]},
//        {SynapticonSlave1, Synapticon, 0x2402, 0, &offset.analog_in2},
//        {SynapticonSlave1, Synapticon, 0x2403, 0, &offset.analog_in3},
//        {SynapticonSlave1, Synapticon, 0x2404, 0, &offset.analog_in4},
//
//        {SynapticonSlave1, Synapticon, 0x2501, 0, &offset.digital_in1},
//        {SynapticonSlave1, Synapticon, 0x2502, 0, &offset.digital_in2},
//        {SynapticonSlave1, Synapticon, 0x2503, 0, &offset.digital_in3},
//        {SynapticonSlave1, Synapticon, 0x2504, 0, &offset.digital_in4},
        {}
};

// PDO entries
static ec_pdo_entry_info_t pdo_entries_Rx[] = {
        /* RxPdo 0x1600 */
        {0x6040, 0x00, 16}, // control word
        {0x6060, 0x00,  8}, // Modes of operation
        {0x60FF, 0x00, 32}, // Target velocity
        {0x607A, 0x00, 32}, // target position
        {0x6071, 0x00, 16}, // Target torque
};

static ec_pdo_entry_info_t pdo_entries_Tx[] = {
        /* TxPdo 0x1A00 */
        {0x6041, 0x00, 16}, // Status word
        {0x606C, 0x00, 32}, // actual velocity
        {0x6064, 0x00, 32}, // actual position
};

// RxPDO
static ec_pdo_info_t RxPDOs[] = {
        /* RxPdo 0x1600 */
        {0x1600,5,pdo_entries_Rx},
};

static ec_pdo_info_t TxPDOs[] = {
        /* TxPdo 0x1A00 */
        {0x1A00,3,pdo_entries_Tx},
};

/*
 * output = values written by master
 * input  = values written by slaves
 */
static ec_sync_info_t device_syncs[] = {
//        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
//        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, RxPDOs , EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 1, TxPDOs , EC_WD_DISABLE},
        {0xFF}
};

void releaseMaster(void){
    if(master){
        std::cout << std::endl;
        std::cout<< "End of program, release master." << std::endl;
        ecrt_release_master(master);
        master = NULL;
    }
}

int configPDO(){
    std::cout << "Configuring PDOs ... " << std::endl;
    domainRx = ecrt_master_create_domain(master);
    if(!domainRx)
        exit(EXIT_FAILURE);
    domainTx = ecrt_master_create_domain(master);
    if(!domainTx)
        exit(EXIT_FAILURE);

    std::cout << "Creating slave configurations ... " << std::endl;

    /*
     * Obtain configuration of slaves
     */
    // slave 1
    if (!(sc[0] = ecrt_master_slave_config(master, SynapticonSlave1, Synapticon))) {
        std::cout << "Failed to get slave 1 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    // slave 2
    if (!(sc[1] = ecrt_master_slave_config(master, SynapticonSlave2, Synapticon))) {
        std::cout << "Failed to get slave 2 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    /*
     * Configuring slaves' PDOs
     */
    // slave 1
    if (ecrt_slave_config_pdos(sc[0], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 1 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 2
    if (ecrt_slave_config_pdos(sc[1], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 2 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (ecrt_domain_reg_pdo_entry_list(domainRx,domain_Rx_reg)){
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }
    if(ecrt_domain_reg_pdo_entry_list(domainTx,domain_Tx_reg)){
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}


// =================== Function =======================

void check_domain_state(void){
    ec_domain_state_t ds = {};
    ec_domain_state_t ds1 = {};

    ecrt_domain_state(domainTx,&ds);
    if(ds.working_counter != domainTx_state.working_counter)
        std::cout << "domainTx: WC " << ds.working_counter;
    if(ds.wc_state != domainTx_state.wc_state)
        std::cout << "domainTx: state " << ds.wc_state;

    domainTx_state = ds;

    ecrt_domain_state(domainRx,&ds1);
    if(ds1.working_counter != domainRx_state.working_counter)
        std::cout << "domainRx: WC " << ds1.working_counter;
    if(ds1.wc_state != domainRx_state.wc_state)
        std::cout << "domainRx: state " << ds1.wc_state;

    domainRx_state = ds1;
}

void check_master_state(void){
    ec_master_state_t ms;
    ecrt_master_state(master,&ms);

    if(ms.slaves_responding != master_state.slaves_responding)
        std::cout << ms.slaves_responding << " slave(s)." << std::endl;
    if(ms.al_states != master_state.al_states)
        std::cout << "AL state : " << ms.al_states << std::endl;
    if(ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up":"down");
    master_state = ms;
}

void check_slave_config_states(void)
{
    ec_slave_config_state_t s[2];

    for (int j = 0; j < 2; j++){
        ecrt_slave_config_state(sc[j],&s[j]);
        if (s[j].al_state != sc_state[j].al_state)
            printf("Slave %d: State 0x%02X.\n",j,s[j].al_state);
        if(s[j].online != sc_state[j].online)
            printf("Slave %d: %s.\n",j,s[j].online ? "online" : "offline");
        if(s[j].operational != sc_state[j].operational)
            printf("slave %d: %soperational.\n",j,s[j].operational ? "":"Not ");
        sc_state[j] = s[j];
    }
}

int ActivateMaster(void){
    int ret;
    std::cout << "Requesting master ... " << std::endl;

    if(master)
        return 0;

    master = ecrt_request_master(0);
    if(!master){
        return -1;
    }

    configPDO();

    std::cout << "Activating master... " << std::endl;

    if (ecrt_master_activate(master)){
        exit((EXIT_FAILURE));
        std::cout << "Activating master...failed. " << std::endl;
    }

    if (!(domainTx_pd = ecrt_domain_data(domainTx))){
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    if (!(domainRx_pd = ecrt_domain_data(domainRx))){
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    std::cout << "Activating master...success. " << std::endl;

    return 0;
}

// =================== Thread   =======================

void cyclic_task(void);
#endif //ROBOT_ROBOT_H
