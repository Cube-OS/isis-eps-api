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
use i2c_rs::{Command, Connection as I2c};

use std::{time::Duration, intrinsics::size_of, mem::size_of};
use crate::objects::*;
use failure::Fail;
use std::convert::From;
use cubeos_service::Error;
use std::mem::*;

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
    #[fail(display = "I2C Error")]
    I2CError2(u8),
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

/// All Errors in EpsError are converted to Error::ServiceError(u8)
impl From<EpsError> for Error {
    fn from(e: EpsError) -> Error {
        match e {
            EpsError::Err => Error::ServiceError(0),
            EpsError::I2CError(io) => Error::from(io),
            EpsError::I2CError2(io) => Error::Io(io),
            EpsError::I2CSet => Error::ServiceError(1),
            EpsError::TransferError => Error::ServiceError(2),
            EpsError::InvalidInput => Error::ServiceError(3),
            EpsError::Bincode(io) => Error::Bincode(io),
            EpsError::Rejected => Error::ServiceError(4),
            EpsError::InvalidCommandCode => Error::ServiceError(5),
            EpsError::ParameterMissing => Error::ServiceError(6),
            EpsError::Parameterinvalid => Error::ServiceError(7),
            EpsError::UnavailableMode => Error::ServiceError(8),
            EpsError::InvalidSystemType => Error::ServiceError(9),
            EpsError::InternalProcessing => Error::ServiceError(10),
            // _ => Error::ServiceError(0),
        }
    }
}

impl From<Error> for EpsError {
    fn from(e: Error) -> EpsError {
        match e {
            Error::ServiceError(0) => EpsError::Err,
            Error::Io(io) => EpsError::I2CError2(io),
            Error::ServiceError(1) => EpsError::I2CSet,
            Error::ServiceError(2) => EpsError::TransferError,
            Error::ServiceError(3) => EpsError::InvalidInput,
            Error::Bincode(io) => EpsError::Bincode(io),
            Error::ServiceError(4) => EpsError::Rejected,
            Error::ServiceError(5) => EpsError::InvalidCommandCode,
            Error::ServiceError(6) => EpsError::ParameterMissing,
            Error::ServiceError(7) => EpsError::Parameterinvalid,
            Error::ServiceError(8) => EpsError::UnavailableMode,
            Error::ServiceError(9) => EpsError::InvalidSystemType,
            Error::ServiceError(10) => EpsError::InternalProcessing,
            _ => EpsError::Err,
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
        0x80 => Ok(()),
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
}

impl EPS {
    // Basic function to initialise an instance of the EpsStruct 
    pub fn new(i2c_path: String, i2c_addr: u16) -> EpsResult<Self> {
        Ok(Self{
            i2c: I2c::from_path(&i2c_path,i2c_addr),
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
       
        // #[cfg(feature = "debug")]
        println!{"Eps Ping Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                // #[cfg(feature = "debug")]
                println!{"Eps Ping Response{:?}",x};
                match_stat(x[4])
            }
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
        
        #[cfg(feature = "debug")]
        println!{"System Reset Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"System Reset Response{:?}",x};
                match_stat(x[4])
            }
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
        
        #[cfg(feature = "debug")]
        println!{"Shutdown All Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Shutdown All Response{:?}",x};
                match_stat(x[4])
            }
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
        
        #[cfg(feature = "debug")]
        println!{"Watchdog Reset Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Watchdog Reset Response{:?}",x};
                match_stat(x[4])
            }
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
        let group_bytes = eps_bitflag.to_le_bytes(); // use little endian for ISIS{

        // e.g. 0b1010011 (=0x0503, decimal 83). This switches output bus channels 0, 1, 4 and 6
        let data:Vec<u8> = [&[ALL_IVID, cmd_code, OVERRIDE_BID], &group_bytes[..]].concat();

        // let data:Vec<u8> = [ALL_IVID, cmd_code, OVERRIDE_BID, group_bytes[0], group_bytes[1]].to_vec();
        let command = Command{cmd, data};
        // Send command
        let rx_len = 5;
        let delay = Duration::from_millis(50);

        #[cfg(feature = "debug")]
        println!{"Set GroupOutput Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Set GroupOutput Response {:?}",x};
                match_stat(x[4])
            }
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

        #[cfg(feature = "debug")]
        println!{"Set SingleOutput Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Set SingleOutput Response {:?}",x};
                match_stat(x[4])
            }
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

        #[cfg(feature = "debug")]
        println!{"Mode Switch Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            // The (5th byte) responsed need to be parsed with match_stat
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Mode Switch Response {:?}",x};
                match_stat(x[4])
            }
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

        #[cfg(feature = "debug")]
        println!{"System Status Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"System Status Response {:?}", x};
                match match_stat(x[4]){
                    // Ok(()) => Ok(bincode::deserialize::<SystemStatus>(&x[5..])?),
                    Ok(()) => Ok(SystemStatus::from(x)),
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

        #[cfg(feature = "debug")]
        println!{"OverCurrent Status Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"OverCurrent Status Response {:?}", x};
                match match_stat(x[4]){
                    Ok(()) => Ok(OverCurrentFaultState::from(x)),
                    // Ok(()) => Ok(bincode::deserialize::<OverCurrentFaultState>(&x[6..50])?),
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

        #[cfg(feature = "debug")]
        println!{"ABF State {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"ABF State Cmd {:?}", x};
                match match_stat(x[4]){
                    Ok(()) => Ok(ABFState::from(x)),
                    // Ok(()) => Ok(bincode::deserialize::<ABFState>(&x[6..8])?),
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
                    Ok(()) => Ok(PBUHk::from(x[6..].to_vec())),
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

        #[cfg(feature = "debug")]
        println!{"PIU HK Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"PIU HK Response {:?}", x};
                match match_stat(x[4]){
                    Ok(()) => Ok(PIUHk::from(x)),
                    // One reseved byte. Starting from the 6th byte
                    // Ok(()) => Ok(bincode::deserialize::<PIUHk>(&x[6..184])?),
                    Err(e) => Err(e),
                }                 
            }
            Err(_e) => Err(EpsError::TransferError),
        }

    } 

    pub fn set_config_para(&self, typ_stid: StID, id: u16, rx_len: u8, input: Vec<u8>) -> EpsResult<Vec<u8>> {
        let cmd: u8 = match_st_id(typ_stid);

        let id = id.to_le_bytes();
        let mut data: Vec<u8> = [ALL_IVID, SET_CONFIG_PARA, OVERRIDE_BID, id[0], id[1]].to_vec();
        data.append(&input);

        let command = Command{cmd,data};        
        
        let delay = Duration::from_millis(50);

        #[cfg(feature = "debug")]
        println!{"System Config Cmd{:?}",command};
        rx_len = rx_len + 8;
        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"System Config Response {:?}",x};
                match match_stat(x[4]){
                    Ok(()) => Ok(x[6..].to_vec()),
                    Err(e) => Err(e),
                }                 
            }            
            Err(_e) => Err(EpsError::TransferError),
        }
    }
    
    pub fn get_config_para(&self, typ_stid: StID, id:u16, rx_len:u8) -> EpsResult<Vec<u8>> {
        let cmd: u8 = match_st_id(typ_stid);

        let id = id.to_le_bytes();
        let data: Vec<u8> = [ALL_IVID, GET_CONFIG_PARA, OVERRIDE_BID, id[0], id[1]].to_vec();

        let command = Command{cmd,data};        
        
        let delay = Duration::from_millis(50);

        #[cfg(feature = "debug")]
        println!{"System Config Cmd{:?}",command};
        rx_len = rx_len + 8;
        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"System Config Response {:?}",x};
                match match_stat(x[4]){
                    Ok(()) => Ok(x[6..].to_vec()),
                    Err(e) => Err(e),
                }                 
            }            
            Err(_e) => Err(EpsError::TransferError),
        }

    }

    pub fn set_ch_startup_ena_bf(&self, typ_stid: STID, input: u32) -> EpsResult<Vec<u8>> {
        let id = 0x6002;
        let rx_len = size_of::<u32>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_startup_ena_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
    let id = 0x6002;
    let rx_len = size_of::<u32>();
    self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_key(&self, typ_stid: STID,input:u32) -> EpsResult<Vec<u8>> {
        let id = 0x6003;
        let rx_len = size_of::<u32>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_key(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6003;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_eba_bf(&self, typ_stid: STID,input:u32) -> EpsResult<Vec<u8>> {
        let id = 0x6004;
        let rx_len = size_of::<u32>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_eba_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6004;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }


    pub fn set_ch_latchoff_key(&self, typ_stid: STID,input:u32) -> EpsResult<Vec<u8>> {
        let id = 0x6005;
        let rx_len = size_of::<u32>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }


    pub fn get_ch_latchoff_key(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6005;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ttc_wdg_timeout(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4000;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ttc_wdg_timeout(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4000;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }
    pub fn set_ttc_wdg_timeout_key(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4001;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }
    pub fn get_ttc_wdg_timeout_key(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4001;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4002;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4002;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay1(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4003;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay1(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4003;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay2(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4004;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay2(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4004;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay3(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4005;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay3(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4005;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay4(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4006;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }    

    pub fn get_ch_startup_delay4(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4006;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay5(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4007;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay5(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4007;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay6(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4008;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay6(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4008;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay7(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4009;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay7(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4009;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay8(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400A;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay8(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400A;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay9(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400B;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay9(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400B;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay10(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400C;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay10(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400C;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay11(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400D;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay11(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400D;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay12(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400E;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay12(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400E;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay13(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x400F;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay13(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x400F;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay14(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4010;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay14(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4010;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay15(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4011;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay15(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4011;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay16(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4012;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay16(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4012;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay17(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4013;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay17(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4013;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay18(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4014;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay18(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4014;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay19(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4015;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay19(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4015;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay20(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4016;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay20(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4016;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay21(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4017;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay21(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4017;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay22(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4018;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay22(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4018;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay23(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4019;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay23(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4019;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay24(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401A;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay24(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401A;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay25(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401B;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay25(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401B;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay26(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401C;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay26(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401C;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay27(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401D;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay27(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401D;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay28(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401E;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay28(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401E;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay29(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x401F;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay29(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x401F;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay30(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4020;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay30(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4020;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_startup_delay31(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4021;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_startup_delay31(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4021;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay(&self, typ_stid: STID,input;u16) -> EpsResult<Vec<u8>> {
        let id = 0x4022;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4022;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay1(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4023;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay1(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4023;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay2(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4024;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay2(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4024;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay3(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4025;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay3(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4025;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay4(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4026;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay4(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4026;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay5(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4027;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay5(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4027;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay6(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4028;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay6(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4028;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay7(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4029;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay7(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4029;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay8(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402A;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay8(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402A;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay9(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402B;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay9(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402B;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay10(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402C;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay10(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402C;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay11(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402D;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay11(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402D;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay12(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402E;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay12(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402E;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay13(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x402F;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay13(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x402F;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay14(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4030;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay14(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4030;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay15(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4031;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay15(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4031;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay16(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4032;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay16(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4032;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay17(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4033;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay17(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4033;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ch_latchoff_delay18(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4034;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay18(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4034;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay19(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4035;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay19(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4035;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay20(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4036;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay20(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4036;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay21(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4037;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay21(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4037;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay22(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4038;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay22(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4038;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay23(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4039;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay23(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4039;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay24(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403A;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay24(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403A;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay25(&self, typ_stid: STID,input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403B;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay25(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403B;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay26(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403C;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay26(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403C;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay27(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403D;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay27(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403D;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ch_latchoff_delay28(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403E;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay28(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403E;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn set_ch_latchoff_delay29(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x403F;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay29(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x403F;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ch_latchoff_delay30(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4040;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }
    
    pub fn get_ch_latchoff_delay30(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4040;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ch_latchoff_delay31(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4041;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ch_latchoff_delay31(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4041;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_safety_volt_lothr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4042;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_safety_volt_lothr(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4042;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_safety_volt_hithr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4043;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_safety_volt_hithr(&self, typ_stid: STID, input:u16) -> EpsResult<Vec<u8>> {
        let id = 0x4043;
        let rx_len = size_of::<u16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_lothr_bp1_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3000;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp1_heater(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3000;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_lothr_bp2_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3001;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp2_heater(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3001;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_lothr_bp3_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3002;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp3_heater(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3002;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_hithr_bp1_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3003;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp1_heater(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3003;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_hithr_bp2_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3004;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp2_heater(&self, typ_stid: STID,input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3004;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_hithr_bp3_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3005;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp3_heater(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3005;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_hithr_bp3_heater(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3005;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_lothr_bp1_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3006;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp1_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3006;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_lothr_bp2_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3007;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp2_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3007;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_lothr_bp3_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3008;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_lothr_bp3_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3008;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_hithr_bp1_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3009;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp1_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3009;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_hithr_bp2_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300A;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp2_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300A;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_hithr_bp3_unbal(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300B;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_hithr_bp3_unbal(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300B;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_mcu_temp_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300C;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_mcu_temp_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300C;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_mcu_temp_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300D;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_mcu_temp_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300D;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_mcu_temp_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300E;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_mcu_temp_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300E;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp1_temp1_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x300F;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp1_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x300F;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp1_temp2_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3010;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp2_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3010;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp1_temp3_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3011;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp3_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3011;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp2_temp1_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3012;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp1_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3012;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_bp2_temp2_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3013;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp2_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3013;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_bp2_temp3_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3014;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp3_bias(&self, typ_stid: STID,input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3014;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp1_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3015;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp1_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3015;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp2_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3016;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp2_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3016;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp3_bias(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3017;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp3_bias(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3017;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp1_temp1_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3018;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp1_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3018;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len input.to_le_bytes())
    }


    pub fn get_bp1_temp2_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3019;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp2_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3019;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_bp1_temp3_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301A;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp3_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301A;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp2_temp1_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301B;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp1_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301B;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }
    
    pub fn get_bp2_temp3_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301B;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }
    
    pub fn set_bp2_temp3_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301B;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }
   
    pub fn get_bp2_temp3_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301D;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp3_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301D;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp1_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301E;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp1_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301E;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp2_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x301F;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp2_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x301F;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp3_premul(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3020;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp3_premul(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3020;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_bp1_temp1_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3021;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp1_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3021;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp1_temp2_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3022;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp2_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3022;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }

    pub fn get_bp1_temp3_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3023;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp1_temp3_posdiv(&self, typ_stid: STID,input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3023;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp2_temp1_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3024;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp1_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3024;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }
    
    pub fn get_bp2_temp2_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3025;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp2_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3025;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp2_temp3_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3026;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp2_temp3_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3026;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp1_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3027;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp1_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3027;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_bp3_temp2_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3028;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp2_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3028;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_bp3_temp3_posdiv(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3029;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_bp3_temp3_posdiv(&self, typ_stid: STID, input:i16) -> EpsResult<Vec<u8>> {
        let id = 0x3029;
        let rx_len = size_of::<i16>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_board_identifier(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2000;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_board_identifier(&self, typ_stid: STID, input:u8) -> EpsResult<Vec<u8>> {
        let id = 0x2000;
        let rx_len = size_of::<u8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_board_identifier_key(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2001;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_board_identifier_key(&self, typ_stid: STID, input:u8) -> EpsResult<Vec<u8>> {
        let id = 0x2001;
        let rx_len = size_of::<u8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_ravg_strength_p2(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2000;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_ravg_strength_p2(&self, typ_stid: STID, input:u8) -> EpsResult<Vec<u8>> {
        let id = 0x2000;
        let rx_len = size_of::<u8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_auto_heat_ena_bp1(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1001;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_heat_ena_bp1(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1001;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_auto_heat_ena_bp2(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1002;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_heat_ena_bp2(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1002;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_auto_heat_ena_bp3(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1003;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_heat_ena_bp3(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1003;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_auto_bal_ena_bp1(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1004;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_bal_ena_bp1(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1004;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_auto_bal_ena_bp2(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1005;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_bal_ena_bp2(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1005;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_auto_bal_ena_bp3(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1006;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_auto_bal_ena_bp3(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1006;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd1_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1007;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd1_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1007;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd2_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1008;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd2_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1008;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd3_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1009;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd3_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1009;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_vd4_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x100A;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd4_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x100A;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len,input.to_le_bytes())
    }


    pub fn get_vd5_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x100B;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd5_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x100B;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input:i8)
    }

    pub fn get_vd6_always_ena(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x100C;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd6_always_ena(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x100C;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


    pub fn get_vd1_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x100E;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd1_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x100E;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd2_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x100F;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd2_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x100F;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd3_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1010;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd3_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1010;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd4_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1011;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd4_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1011;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd5_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1012;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd5_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1012;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }

    pub fn get_vd6_always_disa(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1013;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn set_vd6_always_disa(&self, typ_stid: STID, input:i8) -> EpsResult<Vec<u8>> {
        let id = 0x1013;
        let rx_len = size_of::<i8>();
        self.set_config_para(typ_stid, id, rx_len, input.to_le_bytes())
    }


//Above is for read and write. Below is read only.




    pub fn get_ch_force_ena_use_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6809;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ch_startup_ena_use_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680A;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ch_latch_off_ena_use_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680B;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd1_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680C;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd2_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680D;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd3_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680E;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd4_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x680F;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd5_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6810;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_vd6_alloc_ch_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6811;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_swci_ch_cmd_ena_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6813;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_swci_ch_cmd_disa_bf(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x6814;
        let rx_len = size_of::<u32>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ttc_i2c_slave_addr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4800;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_conf_nvm_save_cntr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4801;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_conf_nvm_save_chks(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4802;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cause(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4803;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cntr_pwron(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4804;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cntr_wdg(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4805;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cntr_cmd(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4806;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cntr_mcu(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4807;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_cntr_emlopo(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4808;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_rst_code_mcu_raw(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4809;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_emlopo_volt_lothr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480A;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_emlopo_volt_hithr(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480B;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_emlopo_period(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480C
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_safety_volt_lothr_used(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480D;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_safety_volt_hithr_used(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480E;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_safety_linger(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x480F;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ttc_wdg_timout_used(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4810;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ttc_prevcmd_elapsed(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x4811;
        let rx_len = size_of::<u16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_adc_mcu_temp_v25_t30(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3800;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_adc_mcu_temp_v25_t85(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x3801;
        let rx_len = size_of::<i16>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_stid(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2800;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_ivid(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2801;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_bid_used(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2802;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_boot_resume_short(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x2803;
        let rx_len = size_of::<u8>();
        self.get_config_para(typ_stid, id, rx_len)
    }

    pub fn get_conf_param_changed(&self, typ_stid: STID) -> EpsResult<Vec<u8>> {
        let id = 0x1800;
        let rx_len = size_of::<i8>();
        self.get_config_para(typ_stid, id, rx_len)
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
        let param_size = match para_id_bytes[1] {
            0x10 => 1, 
            0x20 => 1,  
            0x30 => 2, 
            0x40 => 2, 
            0x50 => 4, 
            0x60 => 4, 
            0x70 => 4, 
            0x80 => 8, 
            0x90 => 8, 
            0xA0 => 8, 
            _=> return Err(EpsError::InvalidInput),
        }; 

        // Send command
        let rx_len = 8 + param_size;
        let delay = Duration::from_millis(50);

        #[cfg(feature = "debug")]
        println!{"System Config Cmd{:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"System Config Response {:?}",x};
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

        #[cfg(feature = "debug")]
        println!{"Reset All Config Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Reset All Config Response {:?}", x};
                match_stat(x[4])
            }
            Err(_e) => Err(EpsError::TransferError),
        }
    }
     
    // Correct the unit’s unix time with the specified amount of seconds.
    // unix time value is returned as part of the “0x40 (0x41) – Get System Status” response, 
    pub fn correct_time(&self, typ_stid: StID, time_correction: i32) -> EpsResult<()> {
        
        let cmd_code: u8 = CORRECT_TIME;
        let cmd: u8 = match_st_id(typ_stid);
        pub fn get_bp1_temp1_premul(&self, typ_stid: StID) -> EpsResult<Vec<u8>> {
            let cmd: u8 = match_st_id(typ_stid);
    
            let id = 0x3018_i16.to_le_bytes();
            let data: Vec<u8> = [ALL_IVID, GET_CONFIG_PARA, OVERRIDE_BID, id[0], id[1]].to_vec();
    
            let command = Command{cmd,data};
            let rx_len = 8+size_of::<i16>();
            
            let delay = Duration::from_millis(50);
    
            #[cfg(feature = "debug")]
            println!{"System Config Cmd{:?}",command};
    
            match self.i2c.transfer(command, rx_len, delay) {
                Ok(x) => {
                    #[cfg(feature = "debug")]
                    println!{"System Config Response {:?}",x};
                    match match_stat(x[4]){
                        Ok(()) => Ok(x[6..].to_vec()),
                        Err(e) => Err(e),
                    }                 
                }            
                Err(_e) => Err(EpsError::TransferError),
            }
    
        }      let command = Command{cmd, data};

        let rx_len = 1;
        let delay = Duration::from_millis(50);

        #[cfg(feature = "debug")]
        println!{"Correct Time Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Correct Time Response {:?}", x};
                match_stat(x[4])
            }
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

        #[cfg(feature = "debug")]
        println!{"Reset All Counters Cmd {:?}",command};

        match self.i2c.transfer(command, rx_len, delay) {
            Ok(x) => {
                #[cfg(feature = "debug")]
                println!{"Reset All Counters Response {:?}", x};
                match_stat(x[4])
            }
            Err(_e) => Err(EpsError::TransferError),
        }
    }

}


