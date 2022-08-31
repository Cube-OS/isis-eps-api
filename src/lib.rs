// Dependancies
use rust_i2c::{Command, Connection as I2c};
use std::cell::RefCell;
use std::time::Duration;
use std::thread;
use serial::*;
use isis_eps_service_v2::*; // move git address to cubeos

// ID's
const PDU_STID: u8 = 0x11;
const PBU_STID: u8 = 0x12;
const PCU_STID: u8 = 0x13;
const PIU_STID: u8 = 0x1A;
const OVERRIDE_STID: u8 = 0x00;
const ALL_IVID: u8 = 0x06;
const OVERRIDE_IVID: u8 = 0x00;
const PDU_BID: u8 = 0x00;
const PBU_BID: u8 = 0x00;
const PCU_BID: u8 = 0x00;
const OVERRIDE_BID: u8 = 0x00;
// Operational command 
const SYS_RESET: u8 = 0xAA;
const NO_OP: u8 = 0x02;
const CANCEL_OP: u8 = 0x04;
const WATCHDOG: u8 = 0x06;
const CORRECT_TIME: u8 = 0x08;
// Bus group command
const OUTPUT_BUS_GROUP_ON: u8 = 0x10;
const OUTPUT_BUS_GROUP_OFF: u8 = 0x12;
const OUTPUT_BUS_GROUP_STATE: u8 = 0x14;
const OUTPUT_BUS_CHANNEL_ON: u8 = 0x16;
const OUTPUT_BUS_CHANNEL_OFF: u8 = 0x18;
// Mode switch command 
const SWITCH_TO_NOMINAL_MODE: u8 = 0x30;
const SWITCH_TO_SAFETY_MODE: u8 = 0x32;
// Data request commands
const GET_SYS_STATUS: u8 = 0x40;
const GET_PDU_OC_FAULT_STATE: u8 = 0x42;
const GET_PBU_ABF_PLACED_STATE: u8 = 0x44;
const GET_PDU_HK_DATA_RAW: u8 = 0x50;
const GET_PDU_HK_DATA_ENG: u8 = 0x52;
const GET_PDU_HK_DATA_AVRG: u8 = 0x54;
const GET_PBU_HK_DATA_RAW: u8 = 0x60;
const GET_PBU_HK_DATA_ENG: u8 = 0x62;
const GET_PBU_HK_DATA_AVRG: u8 = 0x64;
const GET_PCU_HK_DATA_RAW: u8 = 0x70;
const GET_PCU_HK_DATA_ENG: u8 = 0x72;
const GET_PCU_HK_DATA_AVRG: u8 = 0x74;
// Config commands
const GET_CONFIG_PARA: u8 = 0x82;
const SET_CONFIG_PARA: u8 = 0x84;
const RESET_CONFIG_PARA: u8 = 0x86;
const RESET_CONFIG: u8 = 0x90;
const LOAD_CONFIG: u8 = 0x92;
const SAVE_CONFIG: u8 = 0x94;
// Data request commands
const GET_PIU_HK_DATA_RAW: u8 = 0xA0;
const GET_PIU_HK_DATA_ENG: u8 = 0xA2;
const GET_PIU_HK_DATA_AVRG: u8 = 0xA4;

// Input enumerations

// STID 
pub enum STID {
    PduStid,
    PbuStid,
    PcuStid,
    PiuStid,
    OverrideStid,
}

// Output Bus Group 
pub enum BUS_GROUP {
    BusGroupOn,
    BusGroupOff,
    BusGroupState,
}
// Output Bus Channel
pub enum BUS_CHANNEL {
    BusChannelOn,
    BusChannelOff,
}

pub enum MODE_SWITCH {
    Nominal,
    Safety,
}

pub enum DATA_REQUEST {
    SystemStatus, // Relevant for iEPS Compact
    PDUOvercurrent, // Relevant for iEPS Compact
    PBUABFPlacedState, // Relevant for iEPS Compact
    PDURawHK,
    PDUEngHK,
    PDUAvgHK,
    PBURawHK,
    PBUEngHK,
    PBUAvgHK,
    PCURawHK,
    PCUEngHK,
    PCUAvgHK,
    PIURawHK, // Relevant for iEPS Compact
    PIUEngHK, // Relevant for iEPS Compact
    PIUAvgHK, // Relevant for iEPS Compact
}

// Error list
pub enum Error {
    #[fail(display = "Transfer error")]
    TransferError,
    #[fail(display = "Bitflag error")]
    BitflagError,
    // STAT Errors
    #[fail(display = "Rejected error")]
    RejectedError,
    #[fail(display = "Rejected: Invalid command code error")]
    RejectedInvalidCommandCodeError,
    #[fail(display = "Rejected: Parameter missing error")]
    RejectedParameterMissingError,
    #[fail(display = "Rejected: Parameter invalid error")]
    RejectedParameterInvalidError,
    #[fail(display = "Rejected: Unavailable in current mode/configuration error")]
    RejectedUnavailableError,
    #[fail(display = "Rejected: Invalid system type, interface version, or BID error")]
    RejectedInvalidError,
    #[fail(display = "Internal error occurred during processing")]
    InternalProcessingError,

}

// Result type to be implemented
pub type epsResult<T> = Result<T, Error>;

pub struct EPS {
    i2c_connection: I2c,
    buffer: RefCell<Vec<u8>>,
}

// Input/Output Structs
// Could just define these as types
// pub type timeCorrection = i32;
// pub type OBC_BF = u16;
// pub type OBC_IDX = u8;
// pub type ROS = u8;

pub struct addInput {
    pub timeCorrection: i32,
    pub OBC_BF: u16,
    pub OBC_IDX: u8,
    pub ROS: u8,
}

// Define the complex datatypes here ------------------------------------------
pub struct VIPD {
    VOLT: i16,
    CURR: i16,
    POWE: i16,
}

pub struct CCSD {
    VOLT_IN_MPPT: u16,
    CURR_IN_MPPT: u16,
    VOLT_OU_MPPT: u16,
    CURR_OU_MPPT: u16,
}

pub struct EPSOutput {
    // Most functions only require STAT and the rest is N/A-----
    pub STAT: u8,
    // reserved: u8,-----
    pub VOLT_BRDSUP: u16,
    pub TEMP: u16,
    pub VIP_DIST_INPUT: VIPD,
    pub VIP_BAT_INPUT: VIPD,
    pub STAT_OBC_ON: u16,
    pub STAT_OBC_OCF: u16,
    pub BAT_STAT: u16,
    pub BAT_TEMP2: u16,
    pub BAT_TEMP3: u16,
    pub VOLT_VD0: u16,
    pub VOLT_VD1: u16,
    pub VOLT_VD2: u16,
    pub VIP_OBC01: VIPD,
    pub VIP_OBC02: VIPD,
    pub VIP_OBC03: VIPD,
    pub VIP_OBC04: VIPD,
    pub VIP_OBC05: VIPD,
    pub VIP_OBC06: VIPD,
    pub VIP_OBC07: VIPD,
    pub VIP_OBC08: VIPD,
    pub CC1: CCSD,
    pub CC2: CCSD,
    pub CC3: CCSD,
    // Below here are the daughterboard items, N/A if no daughterboard
    pub VIP_OBC09: VIPD,
    pub VIP_OBC10: VIPD,
    pub VIP_OBC11: VIPD,
    pub VIP_OBC12: VIPD,
    pub VIP_OBC13: VIPD,
    pub VIP_OBC14: VIPD,
    pub CC4: CCSD,
    pub CC5: CCSD,

    // DO I ALSO ADD THE OTHER OUTPUTS FOR MISCELLANEOUS RETURNING FUNCTIONS

}

// Most other functions return the STAT parameter. Write function here to check the the STAT for the error code
fn matchSTAT(typ: u8) -> epsResult<()> { // is it <T, Error> ?
    match typ {
        0x00 => Ok(),
        0x01 => RejectedError,
        0x02 => RejectedInvalidCommandCodeError,
        0x03 => RejectedParameterMissingError,
        0x04 => RejectedParameterInvalidError,
        0x05 => RejectedUnavailableError,
        0x06 => RejectedInvalidError,
        0x07 => InternalProcessingError,
        // Reserved values: 0x10, 0x20, 0x40
        // NEW 0x80 set when the response is read for the first time
    }
}


// STID match shortcut
fn matchSTID(typ: STID) -> u8 {
    match typ {
        STID::PduStid => PDU_STID,
        STID::PbuStid => PBU_STID,
        STID::PcuStid => PCU_STID,
        STID::PiuStid => PIU_STID,
        STID::OverrideStid => OVERRIDE_STID,
    }
}

impl EPS {

    // USE THIS AS TEST FUNCTION
    pub fn sys_reset(&self, typ: STID) -> epsResult<()> {

        let RST_KEY: u8 = 0xA6; // Reset key
        let CC: u8 = 0xAA; // command code
        let cmd: u8 = matchSTID(typ);

        let data: Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID, RST_KEY].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }
    }

    pub fn no_op(&self, typ:STID) -> epsResult<()> {
        
        let CC: u8 = 0x02;
        let cmd: u8 = matchSTID(typ);
        let data: Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }
    }

    pub fn cancel_op(&self, typ:STID) -> epsResult<()> {
        
        let CC: u8 = 0x04;
        let cmd: u8 = matchSTID(typ);
        let data: Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }
    }

    pub fn watchdog(&self, typ:STID) -> epsResult<()> {
        
        let CC: u8 = 0x06;
        let cmd: u8 = matchSTID(typ);
        let data: Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }
    }

    // This requires a hexadecimal input, is this even useful? (extra input is i32)
    pub fn correct_time(&self, typ: STID, timeCorrection: i32) -> epsResult<()> {
        
        let CC: u8 = 0x08;
        let cmd: u8 = matchSTID(typ);

        let mut timeCorrection_bytes = timeCorrection.to_le_bytes();
        let data = [&[ALL_IVID, CC, OVERRIDE_BID], &timeCorrection_bytes[..]].concat();
        let command = Command{cmd, data};

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }

    }

    // Rejects non-existent bus channels and groups (extra input is u16)
    pub fn bus_group(&self, typ_stid: STID, typ_group: BUS_GROUP, OBC_BF: u16) -> epsResult<()> {
        // Match correct command arg
        let CC: u8 = match typ_group {
            BUS_GROUP::BusGroupOn => OUTPUT_BUS_CHANNEL_ON,
            BUS_GROUP::BusGroupOff => OUTPUT_BUS_GROUP_OFF,
            BUS_GROUP::BusGroupState => OUTPUT_BUS_GROUP_STATE,
        };

        let cmd: u8 = matchSTID(typ_stid);
        let group_bytes = OBC_BF.to_le_bytes(); // use little endian for ISIS
        let data:Vec<u8> = [&[ALL_IVID, CC, OVERRIDE_BID], &group_bytes[..]].concat();
        let command = Command{cmd, data};
        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }
    }

    // Non-existent bus channel index will be rejected (so we can check here if this is procked in ISIS and present the error)
    pub fn bus_channel(&self, typ_stid: STID, typ_channel: BUS_CHANNEL, OBC_IDX: u8) -> epsResult<()> {

        // Check if rejection index error occurs within ISIS
        if OBC_IDX > 0x09 {
            Err(Error::BitflagError);
        }

        let CC: u8 = match typ_channel {
            BUS_CHANNEL::BusChannelOn => OUTPUT_BUS_CHANNEL_ON,
            BUS_CHANNEL::BusChannelOff => OUTPUT_BUS_CHANNEL_OFF,
        };

        let cmd: u8 = matchSTID(typ_stid);
        let data:Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID, OBC_IDX].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }

    }

    pub fn mode_switch(&self, typ_stid: STID, typ_mode: MODE_SWITCH) -> epsResult<()> {
        let CC: u8 = match typ_mode {
            MODE_SWITCH::Nominal => SWITCH_TO_NOMINAL_MODE,
            MODE_SWITCH::Safety => SWITCH_TO_SAFETY_MODE,
        };

        let cmd: u8 = matchSTID(typ_stid);
        let data:Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }

    }

    // Data Request Function
    pub fn data_request(&self, typ_stid: STID, typ_data: DATA_REQUEST) -> epsResult<()> {
        let CC: u8 = match typ_data {
            DATA_REQUEST::SystemStatus => GET_SYS_STATUS,
            DATA_REQUEST::PDUOvercurrent => GET_PDU_OC_FAULT_STATE,
            DATA_REQUEST::PBUABFPlacedState => GET_PBU_ABF_PLACED_STATE,
            DATA_REQUEST::PDURawHK => GET_PDU_HK_DATA_RAW,
            DATA_REQUEST::PDUEngHK => GET_PDU_HK_DATA_ENG,
            DATA_REQUEST::PDUAvgHK => GET_PDU_HK_DATA_AVRG,
            DATA_REQUEST::PBURawHK => GET_PBU_HK_DATA_RAW,
            DATA_REQUEST::PBUEngHK => GET_PBU_HK_DATA_ENG,
            DATA_REQUEST::PBUAvgHK => GET_PBU_HK_DATA_AVRG,
            DATA_REQUEST::PCURawHK => GET_PCU_HK_DATA_RAW,
            DATA_REQUEST::PCUEngHK => GET_PCU_HK_DATA_ENG,
            DATA_REQUEST::PCUAvgHK => GET_PCU_HK_DATA_AVRG,
            DATA_REQUEST::PIURawHK => GET_PIU_HK_DATA_RAW,
            DATA_REQUEST::PIUEngHK => GET_PIU_HK_DATA_ENG,
            DATA_REQUEST::PIUAvgHK => GET_PIU_HK_DATA_AVRG,
        };

        let cmd: u8 = matchSTID(typ_stid);
        let data:Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }

    }

    // Configuration commands here 
    // (extra input is u8)
    pub fn read_response_part(&self, typ_stid: STID, ROS: u8) -> epsResult<()> {
        let CC: u8 = 0xC2;
        let cmd: u8 = matchSTID(typ_stid);
        let data:Vec<u8> = [ALL_IVID, CC, OVERRIDE_BID, ROS].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.connection.transfer(command, rx_len, delay) {
            Ok(x) => Ok(x),
            Err(e) => Err(Error::TransferError),
        }

    }


}
