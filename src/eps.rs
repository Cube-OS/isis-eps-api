//
// Copyright (C) 2022 CUAVA, The University of Sydney
//
// Licensed under the Apache License, Version 2.0 (the "License")
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! CubeOS API for interacting with [ISIS ICEPSv2]
// Reference documentation: ISIS Electrical Power System 2 –Software ICD – IVID 7

// The API is contributed by Xueliang Bai <x.bai@sydney.edu.au> 
// and Oscar Wilfred Thomas Ansted <oans4023@uni.sydney.edu.au> on behalf of the
// ARC Training Centre for CubeSats, UAVs & Their Applications (CUAVA) team (www.cuava.com.au)
// at the University of Sydney

// Dependancies
use rust_i2c::{Command, Connection as I2c};
use rust_udp::{Connection as Udp};

use std::time::Duration;
use crate::objects::*;
use failure::Fail;
use serde::*;
// use std::cell::RefCell;
// use std::thread;
// use serial::*;
use juniper::*;
use std::convert::From;
use cubeos_error::Error;

// ID's
const PDU_STID: u8 = 0x11;
const PBU_STID: u8 = 0x12;
const PCU_STID: u8 = 0x13;
const PIU_STID: u8 = 0x1A;
const OVERRIDE_STID: u8 = 0x00;
const ALL_IVID: u8 = 0x06;
// const OVERRIDE_IVID: u8 = 0x00;
// const PDU_BID: u8 = 0x00;
// const PBU_BID: u8 = 0x00;
// const PCU_BID: u8 = 0x00;
const OVERRIDE_BID: u8 = 0x00;

// System Operational command 
const SYS_RESET: u8 = 0xAA;
const NO_OP: u8 = 0x02;
const CANCEL_OP: u8 = 0x04;
const WATCHDOG: u8 = 0x06;
// const CORRECT_TIME: u8 = 0x08;
const CORRECT_TIME: u8 = 0xC4;
const RST_CAUSE_CNTR: u8 = 0xC6;

// Bus Group Operational Command
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
// const GET_PDU_HK_DATA_RAW: u8 = 0x50;
const GET_PDU_HK_DATA_ENG: u8 = 0x52;
const GET_PDU_HK_DATA_AVRG: u8 = 0x54;
// const GET_PBU_HK_DATA_RAW: u8 = 0x60;
const GET_PBU_HK_DATA_ENG: u8 = 0x62;
const GET_PBU_HK_DATA_AVRG: u8 = 0x64;
// const GET_PCU_HK_DATA_RAW: u8 = 0x70;
const GET_PCU_HK_DATA_ENG: u8 = 0x72;
const GET_PCU_HK_DATA_AVRG: u8 = 0x74;

// Config commands
const GET_CONFIG_PARA: u8 = 0x82;
const SET_CONFIG_PARA: u8 = 0x84;
const RESET_CONFIG_PARA: u8 = 0x86;
const RESET_CONFIG_ALL: u8 = 0x90;
const LOAD_CONFIG: u8 = 0x92;
const SAVE_CONFIG: u8 = 0x94;

// Data request commands
// const GET_PIU_HK_DATA_RAW: u8 = 0xA0;
const GET_PIU_HK_DATA_ENG: u8 = 0xA2;
const GET_PIU_HK_DATA_AVRG: u8 = 0xA4;

// Error list
#[derive(Debug, Fail, Clone, PartialEq)]
pub enum EpsError {
    /// Example error
    #[fail(display = "Eps Error")]
    Err,
    /// I2C Error
    #[fail(display = "I2C Error")]
    I2CError(std::io::ErrorKind),
    /// I2C Set Error
    #[fail(display = "I2C Set Error")]
    I2CSet,
    #[fail(display = "Transfer error")]
    TransferError,
    #[fail(display = "InvalidInput error")]
    InvalidInput,
    // Errors from deserialization
    #[fail(display = "bincode Error")]
    Bincode(u8),
    // Response Status Information (STAT) Errors
    #[fail(display = "Rejected")]
    Rejected,
    #[fail(display = "Rejected: Invalid command code error")]
    InvalidCommandCode,
    #[fail(display = "Rejected: Parameter missing error")]
    ParameterMissing,
    #[fail(display = "Rejected: Parameter invalid error")]
    Parameterinvalid,
    #[fail(display = "Rejected: Unavailable in current mode/configuration error")]
    UnavailableMode,
    #[fail(display = "Rejected: Invalid system type, interface version, or BID error")]
    InvalidSystemType,
    #[fail(display = "Internal error occurred during processing")]
    InternalProcessing,
}

/// All Errors in EpsError are converted to cubeos_error::Error::ServiceError(u8)
impl From<EpsError> for Error {
    fn from(e: EpsError) -> cubeos_error::Error {
        match e {
            EpsError::Err => cubeos_error::Error::ServiceError(0),
            EpsError::I2CError(io) => cubeos_error::Error::from(io),
            EpsError::I2CSet => cubeos_error::Error::ServiceError(1),
            EpsError::TransferError => cubeos_error::Error::ServiceError(2),
            EpsError::InvalidInput => cubeos_error::Error::ServiceError(3),
            EpsError::Bincode(io) => cubeos_error::Error::Bincode(io),
            EpsError::Rejected => cubeos_error::Error::ServiceError(4),
            EpsError::InvalidCommandCode => cubeos_error::Error::ServiceError(5),
            EpsError::ParameterMissing => cubeos_error::Error::ServiceError(6),
            EpsError::Parameterinvalid => cubeos_error::Error::ServiceError(7),
            EpsError::UnavailableMode => cubeos_error::Error::ServiceError(8),
            EpsError::InvalidSystemType => cubeos_error::Error::ServiceError(9),
            EpsError::InternalProcessing => cubeos_error::Error::ServiceError(10),
            // _ => cubeos_error::Error::ServiceError(0),
        }
    }
}

impl From<bincode::Error> for EpsError {
    fn from(b: bincode::Error) -> EpsError {
        match *b {
            bincode::ErrorKind::Io(_) => EpsError::Bincode(0),
            bincode::ErrorKind::InvalidUtf8Encoding(_) => EpsError::Bincode(1),
            bincode::ErrorKind::InvalidBoolEncoding(_) => EpsError::Bincode(2),
            bincode::ErrorKind::InvalidCharEncoding => EpsError::Bincode(3),
            bincode::ErrorKind::InvalidTagEncoding(_) => EpsError::Bincode(4),
            bincode::ErrorKind::DeserializeAnyNotSupported => EpsError::Bincode(5),
            bincode::ErrorKind::SizeLimit => EpsError::Bincode(6),
            bincode::ErrorKind::SequenceMustHaveLength => EpsError::Bincode(7),
            bincode::ErrorKind::Custom(_) => EpsError::Bincode(8),            
        }
    }
}

// Most other functions return the STAT parameter. Write function here to check the the STAT for the error code
fn match_stat(typ: u8) -> EpsResult<()> { // is it <T, Error> ?
    match typ {
        0x00 => Ok(()),
        0x01 => Err(EpsError::Rejected),
        0x02 => Err(EpsError::InvalidCommandCode),
        0x03 => Err(EpsError::ParameterMissing),
        0x04 => Err(EpsError::Parameterinvalid),
        0x05 => Err(EpsError::UnavailableMode),
        0x06 => Err(EpsError::InvalidSystemType),
        _ => Err(EpsError::InternalProcessing),
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

//To-do: Add battery status enum
//Table 3-18: Battery Pack Status Bitfield Definition

// Result type to be implemented
pub type EpsResult<T> = Result<T, EpsError>;

pub struct EPS {
    i2c: I2c,
    udp_connection: Udp,
}

impl EPS {
    // Basic function to initialise an instance of the EpsStruct 
    pub fn new(
        i2c_path: String,
        i2c_addr: u16,
        // // API's Listener Address
        udp_path: String,
        // // Payload Address
        udp_to: String,
    ) -> EpsResult<Self> {
        Ok(Self{
            i2c: I2c::from_path(&i2c_path,i2c_addr),
            udp_connection: Udp::from_path(udp_path, udp_to),
        })
    }


    // No-operation. Check system availability, without changing anything 
    pub fn eps_ping(&self, typ_stid:StID) -> EpsResult<()> {

        let cmd_code: u8 = NO_OP;
        let cmd: u8 = match_st_id(typ_stid);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command    

        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }
    
    // Software reset. A reply to this command will not always be retrievable (system will shut down after this)
    pub fn sys_reset(&self, typ_stid: StID, ret_key: u8) -> EpsResult<()> {

        // let ret_key: u8 = 0xA6; // Reset key
        let cmd_code: u8 = SYS_RESET; // command code
        let cmd: u8 = match_st_id(typ_stid);
        
        // The value of ret_key needs to be set to 0xA6 for the command to be accepted.
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, ret_key].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // Switches off any command-enable output bus channels. 
    // All force-enable channels will remain enabled.
    pub fn shutdown_all(&self, typ_stid:StID) -> EpsResult<()> {
        
        let cmd_code: u8 = CANCEL_OP;
        let cmd: u8 = match_st_id(typ_stid);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // Resets the watchdog timer keeping the system from performing a reset (0x06)
    // Note tha any traffic with the system implicitly performs a watchdog reset. 
    pub fn watchdog_reset(&self, typ_stid:StID) -> EpsResult<()> {
        
        let cmd_code: u8 = WATCHDOG;
        let cmd: u8 = match_st_id(typ_stid);
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // Turn-on/off output bus channels with bitflag, leave unmarked unaltered. （0x10,0x12,0x14）
    // LSB bit corresponds to bus channel 0 (CH0),
    pub fn set_group_outputs(&self, typ_stid: StID, typ_group: BusGroup, eps_bitflag: u16) -> EpsResult<()> {
        // Match correct command arg
        let cmd_code: u8 = match typ_group {
            BusGroup::BusGroupOn => OUTPUT_BUS_GROUP_ON,
            BusGroup::BusGroupOff => OUTPUT_BUS_GROUP_OFF,
            BusGroup::BusGroupState => OUTPUT_BUS_GROUP_STATE,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let group_bytes = eps_bitflag.to_le_bytes(); // use little endian for ISIS
        // e.g. 0b1010011 (=0x0503, decimal 83). This switches output bus channels 0, 1, 4 and 6
        let data:Vec<u8> = [&[ALL_IVID, cmd_code, OVERRIDE_BID], &group_bytes[..]].concat();

        // let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, group_bytes[0], group_bytes[1]].to_vec();
        let command = Command{cmd, data};
        // Send command
        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

    // Turn a single output bus channel on using the bus channel index. (0x16,0x18)
    // e.g. Index 0 represents channel 0 (CH0)
    pub fn set_single_output(&self, typ_stid: StID, typ_channel: BusChannel, eps_ch_idx: u8) -> EpsResult<()> {

        // Check if rejection index error occurs within ISIS
        // Designed for ICEPSv2 (17 channels), Consider to remove this for larger iEPS modules 
        if eps_ch_idx > 0x10 {
            return Err::<(),EpsError>(EpsError::InvalidInput);
        }

        let cmd_code: u8 = match typ_channel {
            BusChannel::ChannelOn => OUTPUT_BUS_CHANNEL_ON,
            BusChannel::ChannelOff => OUTPUT_BUS_CHANNEL_OFF,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, eps_ch_idx].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    pub fn mode_switch(&self, typ_stid: StID, mode: ModeSwitch) -> EpsResult<()> {
        let cmd_code: u8 = match mode {
            ModeSwitch::Nominal => SWITCH_TO_NOMINAL_MODE,
            ModeSwitch::Safety => SWITCH_TO_SAFETY_MODE,
        };

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // Get EPS System Status
    pub fn system_status(&self, typ_stid: StID) -> EpsResult<SystemStatus> {
        let cmd_code: u8 = GET_SYS_STATUS;

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 36;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<SystemStatus>(&x[5..])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // 0x42  – Get Overcurrent Fault State
    pub fn overcurrent_state(&self, typ_stid: StID) -> EpsResult<OverCurrentFaultState> {
        let cmd_code: u8 = GET_PDU_OC_FAULT_STATE;

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 78;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<OverCurrentFaultState>(&x[6..50])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // 0x44  – Get ABF Placed State
    pub fn abf_state(&self, typ_stid: StID) -> EpsResult<ABFState> {
        let cmd_code: u8 = GET_PBU_ABF_PLACED_STATE;

        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 8;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<ABFState>(&x[6..8])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    }
    
    // 0x52 and 0x54  – Get PDU Housekeeping Data (Engineering and Average Data)
    pub fn pdu_hk(&self, typ_stid: StID, mode: PDUHkSel) -> EpsResult<PDUHk> {
        let cmd_code: u8 = match mode {
            PDUHkSel::PDUEngHK => GET_PDU_HK_DATA_ENG,
            PDUHkSel::PDUAvgHK => GET_PDU_HK_DATA_AVRG,
        };
        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 258;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<PDUHk>(&x[6..168])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // 0x62 and 0x64  – Get PBU Housekeeping Data (Engineering and Average Data)
    pub fn pbu_hk(&self, typ_stid: StID, mode: PBUHkSel) -> EpsResult<PBUHk> {
        let cmd_code: u8 = match mode {
            PBUHkSel::PBUEngHK => GET_PBU_HK_DATA_ENG,
            PBUHkSel::PBUAvgHK => GET_PBU_HK_DATA_AVRG,
        };
        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 84;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<PBUHk>(&x[6..])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // 0x72 and 0x74  – Get PCU Housekeeping Data (Engineering and Average Data)
    pub fn pcu_hk(&self, typ_stid: StID, mode: PCUHkSel) -> EpsResult<PCUHk> {
        let cmd_code: u8 = match mode {
            PCUHkSel::PCUEngHK => GET_PCU_HK_DATA_ENG,
            PCUHkSel::PCUAvgHK => GET_PCU_HK_DATA_AVRG,
        };
        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 72;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<PCUHk>(&x[6..])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    } 

    // 0xA2 and 0xA4  – Get PIU Housekeeping Data (Engineering and Average Data)
    pub fn piu_hk(&self, typ_stid: StID, mode: PIUHkSel) -> EpsResult<PIUHk> {
        let cmd_code: u8 = match mode {
            PIUHkSel::PIUEngHK => GET_PIU_HK_DATA_ENG,
            PIUHkSel::PIUAvgHK => GET_PIU_HK_DATA_AVRG,
        };
        let cmd: u8 = match_st_id(typ_stid);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID].to_vec();
        let command = Command{cmd, data};

        // Send command
        // 116 bytes w/o daughterboard, 274 bytes with daughterboard
        let rx_len = 274;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(bincode::deserialize::<PIUHk>(&x[6..184])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    } 

    // 0x82/0x84/0x86 Get/Set/Reset Configuration commands 
    // XL: Not sure how to handle the return
    pub fn system_config_command(&self, typ_stid: StID, mode: SysConfig1, para_id: u16) -> EpsResult<Vec<u8>> {

        let cmd_code: u8 = match mode {
            SysConfig1::GetConfigParam => GET_CONFIG_PARA,
            SysConfig1::SetConfigParam => SET_CONFIG_PARA,
            SysConfig1::ResetConfigParam => RESET_CONFIG_PARA,
        };
        let cmd: u8 = match_st_id(typ_stid);
        let para_id_bytes = para_id.to_le_bytes(); // use little endian for ISIS
        println!("{:?}", para_id_bytes);
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, para_id_bytes[0], para_id_bytes[1]].to_vec();

        let command = Command{cmd, data};
        let param_size;

        match para_id_bytes[1] {
            0x10 => param_size = 1, 
            0x20 => param_size = 1,  
            0x30 => param_size = 2, 
            0x40 => param_size = 2, 
            0x50 => param_size = 4, 
            0x60 => param_size = 4, 
            0x70 => param_size = 4, 
            0x80 => param_size = 8, 
            0x90 => param_size = 8, 
            0xA0 => param_size = 8, 
            _=> return Err(EpsError::InvalidInput),
        } 

        // Send command
        let rx_len = 8 + param_size;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                match match_stat(x[4]){
                    Ok(()) => Ok(x[6..].to_vec()),
                    Err(e) => Err(e),
                }                 
            }            
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    // 0x90 Reset a parameter to its default hard-coded value. 
    pub fn reset_all_conf(&self, typ_stid: StID, mode: SysConfig2, config_key: u8) -> EpsResult<()> {
       // XL let cmd_code: u8 = RESET_CONFIG;
       let cmd_code: u8 = match mode {
            SysConfig2::ResetAll => RESET_CONFIG_ALL,
            SysConfig2::LoadConfig => LOAD_CONFIG,
            SysConfig2::SaveConfig => SAVE_CONFIG,
        };
        let cmd: u8 = match_st_id(typ_stid);
        // Config key must be 0xA7, any other value will be rejected with a parameter error
        let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, config_key].to_vec();
        let command = Command{cmd, data};

        // Send command
        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }
     
    // Correct the unit’s unix time with the specified amount of seconds.
    // unix time value is returned as part of the “0x40 (0x41) – Get System Status” response, 
    pub fn correct_time(&self, typ_stid: StID, time_correction: i32) -> EpsResult<()> {
        
        let cmd_code: u8 = CORRECT_TIME;
        let cmd: u8 = match_st_id(typ_stid);

        let time_correction_bytes = time_correction.to_le_bytes();
        let data = [&[ALL_IVID, cmd_code, OVERRIDE_BID], &time_correction_bytes[..]].concat();
        let command = Command{cmd, data};

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    //  Write all reset cause counters to zero in persistent memory (0xC6)
    pub fn reset_all_counters(&self, typ_stid:StID, zero_key:u8) -> EpsResult<()> {
        
        let cmd_code: u8 = RST_CAUSE_CNTR;
        let cmd: u8 = match_st_id(typ_stid);

        // Zero key: 0xA7. Any other value causes this command to be rejected with a parameter error
        // XL: Not sure why zero_key is defined as i32 in manual, to be tested
        let data: Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, zero_key].to_vec();
        let command = Command{cmd, data}; // i2c command 

        let rx_len = 5;
        let delay = Duration::from_millis(50);

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => match_stat(x[4]),
            Err(_e) => Err(EpsError::TransferError),
        }
    }

}


