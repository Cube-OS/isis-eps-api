// Dependancies
use rust_i2c::{Command, Connection as I2c};
use std::cell::RefCell;
use std::time::Duration;
use std::thread;
// use serial::*;
use failure::Fail;
use serde::*;

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
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum StID {
    PduStid,
    PbuStid,
    PcuStid,
    PiuStid,
    OverrideStid,
}

// Output Bus Group 
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum BusGroup {
    BusGroupOn,
    BusGroupOff,
    BusGroupState,
}

// Output Bus Channel
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum BusChannel {
    BusChannelOn,
    BusChannelOff,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ModeSwitch {
    Nominal,
    Safety,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum DataRequest {
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
#[derive(Debug, Fail, Serialize, Deserialize, Clone, PartialEq)]
pub enum EpsError {
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
pub type EpsResult<T> = Result<T, EpsError>;

pub struct EPS {
    i2c: I2c,
    buffer: RefCell<Vec<u8>>,
}

// Input/Output Structs
// Could just define these as types
// pub type timeCorrection = i32;
// pub type obc_bf = u16;
// pub type obc_idx = u8;
// pub type ros = u8;

#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct EpsInput {
    pub time_correction: i32,
    pub obc_bf: u16,
    pub obc_idx: u8,
    pub ros: u8,
}

// Define the complex datatypes here ------------------------------------------
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct VIPData {
    volt: i16,
    curr: i16,
    pwr: i16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct CmdCodeSdata {
    volt_in_mppt: u16,
    curr_in_mppt: u16,
    volt_out_mppt: u16,
    curr_out_mppt: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct EpsOutput {
    // Most functions only require STAT and the rest is N/A-----
    pub stat: u8,
    // reserved: u8,-----
    pub volt_brdsup: u16,
    pub temp: u16,
    pub vip_dist_input: VIPData,
    pub vip_bat_input: VIPData,
    pub stat_obc_on: u16,
    pub stat_obc_ocf: u16,
    pub bat_stat: u16,
    pub bat_temp2: u16,
    pub bat_temp3: u16,
    pub volt_vd0: u16,
    pub volt_vd1: u16,
    pub volt_vd2: u16,
    pub vip_obc01: VIPData,
    pub vip_obc02: VIPData,
    pub vip_obc03: VIPData,
    pub vip_obc04: VIPData,
    pub vip_obc05: VIPData,
    pub vip_obc06: VIPData,
    pub vip_obc07: VIPData,
    pub vip_obc08: VIPData,
    pub cc1: CmdCodeSdata,
    pub cc2: CmdCodeSdata,
    pub cc3: CmdCodeSdata,
    // below here are the daughterboard items, n/a if no daughterboard
    pub vip_obc09: VIPData,
    pub vip_obc10: VIPData,
    pub vip_obc11: VIPData,
    pub vip_obc12: VIPData,
    pub vip_obc13: VIPData,
    pub vip_obc14: VIPData,
    pub cc4: CmdCodeSdata,
    pub cc5: CmdCodeSdata,
    // DO I ALSO ADD THE OTHER OUTPUTS FOR MISCELLANEOUS RETURNING FUNCTIONS
}

// Most other functions return the STAT parameter. Write function here to check the the STAT for the error code
fn match_stat(typ: u8) -> EpsResult<()> { // is it <T, Error> ?
    match typ {
        0x00 => Ok(()),
        0x01 => Err(EpsError::RejectedError),
        0x02 => Err(EpsError::RejectedInvalidCommandCodeError),
        0x03 => Err(EpsError::RejectedParameterMissingError),
        0x04 => Err(EpsError::RejectedParameterInvalidError),
        0x05 => Err(EpsError::RejectedUnavailableError),
        0x06 => Err(EpsError::RejectedInvalidError),
        _ => Err(EpsError::InternalProcessingError),
        // Reserved values: 0x10, 0x20, 0x40
        // NEW 0x80 set when the response is read for the first time
    }
}


// STID match shortcut
fn match_st_id(typ: StID) -> u8 {
    match typ {
        StID::PduStid => PDU_STID,
        StID::PbuStid => PBU_STID,
        StID::PcuStid => PCU_STID,
        StID::PiuStid => PIU_STID,
        StID::OverrideStid => OVERRIDE_STID,
    }
}

impl EPS {

    // USE THIS AS TEST FUNCTION
    pub fn sys_reset(&self, typ: StID) -> EpsResult<()> {

        let ret_key: u8 = 0xA6; // Reset key
        let cmd_code: u8 = 0xAA; // command code
        let cmd: u8 = match_st_id(typ);

        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, ret_key].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    pub fn no_op(&self, typ:StID) -> EpsResult<()> {
        
        let cmd_code: u8 = 0x02;
        let cmd: u8 = match_st_id(typ);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command    

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    pub fn cancel_op(&self, typ:StID) -> EpsResult<()> {
        
        let cmd_code: u8 = 0x04;
        let cmd: u8 = match_st_id(typ);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    pub fn watchdog(&self, typ:StID) -> EpsResult<()> {
        
        let cmd_code: u8 = 0x06;
        let cmd: u8 = match_st_id(typ);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // This requires a hexadecimal input, is this even useful? (extra input is i32)
    pub fn correct_time(&self, typ: StID, time_correction: i32) -> EpsResult<()> {
        
        let cmd_code: u8 = 0x08;
        let cmd: u8 = match_st_id(typ);

        let time_correction_bytes = time_correction.to_le_bytes();
        let data = [&[ALL_IVID, cmd_code, OVERRIDE_BID], &time_correction_bytes[..]].concat();
        let command = Command{cmd, data};

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // Rejects non-existent bus channels and groups (extra input is u16)
    pub fn bus_group(&self, typ_stid: StID, typ_group: BusGroup, obc_bf: u16) -> EpsResult<()> {
        // Match correct command arg
        let cmd_code: u8 = match typ_group {
            BusGroup::BusGroupOn => OUTPUT_BUS_CHANNEL_ON,
            BusGroup::BusGroupOff => OUTPUT_BUS_GROUP_OFF,
            BusGroup::BusGroupState => OUTPUT_BUS_GROUP_STATE,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let group_bytes = obc_bf.to_le_bytes(); // use little endian for ISIS
        let data:Vec<u8> = [&[ALL_IVID, cmd_code, OVERRIDE_BID], &group_bytes[..]].concat();
        let command = Command{cmd, data};
        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // Non-existent bus channel index will be rejected (so we can check here if this is procked in ISIS and present the error)
    pub fn bus_channel(&self, typ_stid: StID, typ_channel: BusChannel, obc_idx: u8) -> EpsResult<()> {

        // Check if rejection index error occurs within ISIS
        if obc_idx > 0x09 {
            return Err::<(),EpsError>(EpsError::BitflagError);
        }

        let cmd_code: u8 = match typ_channel {
            BusChannel::BusChannelOn => OUTPUT_BUS_CHANNEL_ON,
            BusChannel::BusChannelOff => OUTPUT_BUS_CHANNEL_OFF,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, obc_idx].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    pub fn mode_switch(&self, typ_stid: StID, typ_mode: ModeSwitch) -> EpsResult<()> {
        let cmd_code: u8 = match typ_mode {
            ModeSwitch::Nominal => SWITCH_TO_NOMINAL_MODE,
            ModeSwitch::Safety => SWITCH_TO_SAFETY_MODE,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // Data Request Function
    pub fn data_request(&self, typ_stid: StID, typ_data: DataRequest) -> EpsResult<()> {
        let cmd_code: u8 = match typ_data {
            DataRequest::SystemStatus => GET_SYS_STATUS,
            DataRequest::PDUOvercurrent => GET_PDU_OC_FAULT_STATE,
            DataRequest::PBUABFPlacedState => GET_PBU_ABF_PLACED_STATE,
            DataRequest::PDURawHK => GET_PDU_HK_DATA_RAW,
            DataRequest::PDUEngHK => GET_PDU_HK_DATA_ENG,
            DataRequest::PDUAvgHK => GET_PDU_HK_DATA_AVRG,
            DataRequest::PBURawHK => GET_PBU_HK_DATA_RAW,
            DataRequest::PBUEngHK => GET_PBU_HK_DATA_ENG,
            DataRequest::PBUAvgHK => GET_PBU_HK_DATA_AVRG,
            DataRequest::PCURawHK => GET_PCU_HK_DATA_RAW,
            DataRequest::PCUEngHK => GET_PCU_HK_DATA_ENG,
            DataRequest::PCUAvgHK => GET_PCU_HK_DATA_AVRG,
            DataRequest::PIURawHK => GET_PIU_HK_DATA_RAW,
            DataRequest::PIUEngHK => GET_PIU_HK_DATA_ENG,
            DataRequest::PIUAvgHK => GET_PIU_HK_DATA_AVRG,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // Configuration commands here 
    // (extra input is u8)
    pub fn read_response_part(&self, typ_stid: StID, ros: u8) -> EpsResult<()> {
        let cmd_code: u8 = 0xC2;
        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, ros].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(_x) => Ok(()),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

}
