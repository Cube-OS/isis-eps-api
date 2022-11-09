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

// Input enumerations
// System Type Identifier (STID)

use serde::*;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum StID {
    // Power Distribution Unit System Type Identifier
    PduStid,
    // Power Battery Unit System Type Identifier
    PbuStid,
    // Power Condition Unit System Type Identifier
    PcuStid,
    // Power Intergration unit System Type Identifier
    PiuStid,
    // over write System Type Identifier (i.e. Stid = 0x00)
    OverrideStid,
}

// Output Bus Group 
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum BusGroup {
    BusGroupOn,
    BusGroupOff,
    BusGroupState,
}

// Output Bus Channel
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum BusChannel {
    ChannelOn,
    ChannelOff,
}

// Used in ModeSwitch (0x30/0x31)
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum ModeSwitch {
    Nominal,
    Safety,
}

// Reset status, used in get system status (0x40)
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum EpsMode {
    Startup,
    Nominal,
    Safety,
    Contigency,
}

// Reset status, used in get system status (0x40)
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum ConfStatus {
    NotAltered,
    Altered,
}

// Reset status, used in get system status (0x40)
// #[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
// pub enum ResetCause {
//     PowerOn,
//     Watchdog,
//     Commanded,
//     EpsUpset,
//     EmergLowPwr,
// }

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum PDUHkSel {
    // PDURawHK,
    PDUEngHK,
    PDUAvgHK,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum PBUHkSel {
    // PBURawHK,
    PBUEngHK,
    PBUAvgHK,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum PCUHkSel {
    // PCURawHK,
    PCUEngHK,
    PCUAvgHK,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum PIUHkSel {
    // PIURawHK,
    PIUEngHK,
    PIUAvgHK, 
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum SysConfig1 {
    GetConfigParam, 
    SetConfigParam, 
    ResetConfigParam,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, GraphQLEnum)]
pub enum SysConfig2 {
    ResetAll, 
    LoadConfig, 
    SaveConfig, 
}
 
// The voltage V - current I - power P datatype (VIPD) raw data. 
// Used in blocks across the HK telemetry.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct VIPRawData {
    volt_raw: i16,
    curr_raw: i16,
    pwr_raw: i16,
}

// The voltage V - current I - power P datatype (VIPD) data. 
// Used in blocks across the HK telemetry.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct VIPData {
    volt: i16,
    curr: i16,
    pwr: i16,
}

// The battery pack raw data (BPD). 
// Used in the PBU HK telemetry
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct BattPackRawData{
    vip_bp_output_raw: VIPRawData,
    stat_bp_raw: u16,
    volt_cell1_raw: u16,
    volt_cell2_raw: u16,
    volt_cell3_raw: u16,
    volt_cell4_raw: u16,
    bat_temp1_raw: u16,
    bat_temp2_raw: u16,
    bat_temp3_raw: u16,
}

// The battery pack data (BPD). 
// Used in the PBU HK telemetry
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct BattPackData{
    vip_bp_output: VIPData,
    stat_bp: u16,
    volt_cell1: i16,
    volt_cell2: i16,
    volt_cell3: i16,
    volt_cell4: i16,
    bat_temp1: i16,
    bat_temp2: i16,
    bat_temp3: i16,
}

//CCD Raw data, the conditioning channel datatype (CCD) for each power conditioning chain  
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct CondChnRawData {
    vip_cc_output_raw: BattPackRawData,
    volt_in_mppt_raw: u16,
    curr_in_mppt_raw: u16,
    volt_out_mppt_raw: u16,
    curr_out_mppt_raw: u16,
}

//CCD data, the conditioning channel datatype for each power conditioning chain  
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct CondChnData {
    vip_cc_output: BattPackData,
    volt_in_mppt: i16,
    curr_in_mppt: i16,
    volt_out_mppt: i16,
    curr_out_mppt: i16,
}

//CCSD raw, Short for conditioning channel datatype (CCD), withou VIP data
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct CondChnShortRawData {
    volt_in_mppt_raw: u16,
    curr_in_mppt_raw: u16,
    volt_out_mppt_raw: u16,
    curr_out_mppt_raw: u16,
}

//CCSD, Short for conditioning channel datatype (CCD), withou VIP data
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct CondChnShortData {
    volt_in_mppt: i16,
    curr_in_mppt: i16,
    volt_out_mppt: i16,
    curr_out_mppt: i16,
}

/* ----------------------------------------------------------------
Query response, STID, IVID, RC, BID and STAT are ignored in the structure. 
Structure takes the 5th offset byte (0 to 4 are fixed) as the first byte of the structure.
*/

// System status information (0x40)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct SystemStatus {
    // 0 = startup; 1 = nominal; 2 = safety; 3 = emergency low power
    mode: u8,
    // Configuration parameters have been changed since the last parameters load/save operation
    conf: u8,
    // Reset cause
    reset_cause: u8,
    // Uptime since system start expressed in seconds.
    uptime: u32,
    //First internal error encountered during the system control cycle
    error: u16,
    // Power-on reset counter since begin of life cycle
    rc_cnt_pwron: u16,
    // Watchdog reset counter since begin of life cycle
    rc_cnt_wdg: u16,
    // Cmd reset counter since begin of life cycle
    rc_cnt_cmd: u16,
    // EPS upset reset counter since begin of life cycle
    rc_cnt_mcu: u16,
    // Lower Power reset counter since begin of life cycle
    rc_cnt_lowpwr: u16,
    // Time elapsed between reception of the previous and this command.
    prevcmd_elapsed: u16,
    // Seconds elapsed since 1970-01-01 00:00:00
    unix_time: u32,
    // Year without century part
    unix_year: u8,
    // Calendar month of UNIX_TIME
    unix_month: u8,
    // Calendar day of UNIX_TIME
    unix_day: u8, 
    // Calendar hour of UNIX_TIME
    unix_hour: u8,
    // Calendar minute of UNIX_minute
    unix_minute: u8,
    // Calendar second of UNIX_second
    unix_second: u8,
}


// Overcurrent Fault State （0x42）
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct OverCurrentFaultState {
    // One reseved byte. Starting from the 6th byte
    // Length of useful data for ICEPSv2 (17 channels), 50bytes
    // Bitflag field indicating channel-on status. 1 means corresponding output bus is enabled
    stat_ch_on: u16,
    // Bitflag field indicating overcurrent fault status
    stat_ch_ext_on: u16,
    // VD0_0, 3.3V
    ocf_cnt_ch00: u16,
    // VD1_0, 5V
    ocf_cnt_ch01: u16,
    // VD1_1, 5V
    ocf_cnt_ch02: u16,
    // VD1_2, 5V
    ocf_cnt_ch03: u16,
    // VD1_3, 3.3V
    ocf_cnt_ch04: u16,
    // VD2_0, 3.3V
    ocf_cnt_ch05: u16,
    // VD2_1, 3.3V
    ocf_cnt_ch06: u16,
    // VD2_2, 3.3V
    ocf_cnt_ch07: u16,
    // VD2_3, 3.3V
    ocf_cnt_ch08: u16,
    // VD0_1, 3.3V 
    ocf_cnt_ch09: u16,
    // VD0_2, 3.3V    
    ocf_cnt_ch10: u16,
    // VD0_3, 3.3V    
    ocf_cnt_ch11: u16,
    // VD3_0, 5.4V (customized)
    ocf_cnt_ch12: u16,
    // VD3_1, 5.4V (customized)
    ocf_cnt_ch13: u16,
    // VD4_0, 12V (customized)
    ocf_cnt_ch14: u16,
    // VD4_1, 12V (customized)
    ocf_cnt_ch15: u16,
    // VD5_0, 28.2V 
    ocf_cnt_ch16: u16,
}

// PBU ABF Placed State (0x44)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct ABFState {
    // One reseved byte. Starting from the 6th byte
    // 0xAB = ABF is placed, 0x00 = ABF is not placed
    pub abf_placed_0: u8,
    // 0xAB = ABF is placed, 0x00 = ABF is not placed
    pub abf_placed_1: u8,
}

// PDU Housekeeping Engineering/Average Data (0x52 and 0x54)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct PDUHk {
    // One reseved byte. Starting from the 6th byte
    // Voltage of internal board supply.
    volt_brdsup: i16,
    // Measured temperature of the MCU
    temp: i16,
    // Input V, I and P data for the unit.
    vip_input: VIPData,
    // Bitflag field indicating channel-on status for output 0 through 15.
    stat_ch_on:u16,
    // Bitflag field indicating channel-on status for output 16 through 31.
    stat_ch_ext_on:u16,
    // Bitflag field indicating overcurrent latch-off fault for output 0 through 15.
    stat_ch_ocf:u16,
    // Bitflag field indicating overcurrent latch-off fault for output 16 through 31.
    stat_ch_ext_ocf:u16,
    // VIPData Output V, I and P of voltage domain 0 - 6
    vip_vd0:VIPData,
    vip_vd1:VIPData,
    vip_vd2:VIPData,
    vip_vd3:VIPData,
    vip_vd4:VIPData,
    vip_vd5:VIPData,
    vip_vd6:VIPData,
    // VIPData output for channel 0 - 16
    // VD0_0, 3.3V
    vip_cnt_ch00: VIPData,
    // VD1_0, 5V
    vip_cnt_ch01: VIPData,
    // VD1_1, 5V
    vip_cnt_ch02: VIPData,
    // VD1_2, 5V
    vip_cnt_ch03: VIPData,
    // VD1_3, 3.3V
    vip_cnt_ch04: VIPData,
    // VD2_0, 3.3V
    vip_cnt_ch05: VIPData,
    // VD2_1, 3.3V
    vip_cnt_ch06: VIPData,
    // VD2_2, 3.3V
    vip_cnt_ch07: VIPData,
    // VD2_3, 3.3V
    vip_cnt_ch08: VIPData,
    // VD0_1, 3.3V 
    vip_cnt_ch09: VIPData,
    // VD0_2, 3.3V    
    vip_cnt_ch10: VIPData,
    // VD0_3, 3.3V    
    vip_cnt_ch11: VIPData,
    // VD3_0, 5.4V (customized)
    vip_cnt_ch12: VIPData,
    // VD3_1, 5.4V (customized)
    vip_cnt_ch13: VIPData,
    // VD4_0, 12V (customized)
    vip_cnt_ch14: VIPData,
    // VD4_1, 12V (customized)
    vip_cnt_ch15: VIPData,
    // VD5_0, 28.2V 
    vip_cnt_ch16: VIPData,
}

// PBU Housekeeping Engineering/Average Data (0x62 and 0x64)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct PBUHk {
    // One reseved byte. Starting from the 6th byte
    // Voltage of internal board supply.
    pub volt_brdsup: i16,
    // °C Measured temperature 
    pub temp: i16,
    pub vip_input: VIPData,
    // Bitflag field showing any raised flags on any battery chain
    pub stat_bu: u16,
    // Battery pack channel information.
    pub bp1: BattPackData,
    pub bp2: BattPackData,
    pub bp3: BattPackData,
}

// PCU Housekeeping Engineering/Average Data (0x72 and 0x74)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct PCUHk {
    // One reseved byte. Starting from the 6th byte
    // Voltage of internal board supply.
    pub volt_brdsup: i16,
    // Measured temperature of the MCU
    pub temp: i16,
    // Onput V, I and P data for the unit.
    pub vip_output: VIPData,
    // CCD Data on conditioning chain.
    pub ccd1: CondChnData,
    pub ccd2: CondChnData,
    pub ccd3: CondChnData,
    pub ccd4: CondChnData,
}

// PIU Housekeeping Engineering/Average Data (0xA2 and 0xA4)
#[derive(Clone, Debug, Default, Serialize, Deserialize, Hash)]
pub struct PIUHk {
    // One reseved byte. Starting from the 6th byte
    // Voltage of internal board supply.
    volt_brdsup: i16,
    // Measured temperature of the MCU
    temp: i16,
    // Input V, I and P input of the distribution part of the unit in raw form.
    vip_dist_input: VIPData,
    // Input V, I and P input of the battery part of the unit 
    vip_batt_input: VIPData,
    // Bitflag field indicating channel-on status for output 0 through 15.
    stat_ch_on:u16,
    // Bitflag field indicating overcurrent latch-off fault for output 0 through 15.
    stat_ch_ocf:u16,
    // Bitflag field indicating BP board status.
    batt_stat:u16,
    // 2 and 4 cell battery pack
    batt_temp2:i16,
    // 2 cell battery pack not used, temp for 4 cell battery pack:
    batt_temp3:i16,
    // Voltage level for domain 0 - 2
    volt_vd0:i16,
    volt_vd1:i16,
    volt_vd2:i16,
    // VIPData output for channel 0 - 16
    // VD0_0, 3.3V
    vip_cnt_ch00: VIPData,
    // VD1_0, 5V
    vip_cnt_ch01: VIPData,
    // VD1_1, 5V
    vip_cnt_ch02: VIPData,
    // VD1_2, 5V
    vip_cnt_ch03: VIPData,
    // VD1_3, 3.3V
    vip_cnt_ch04: VIPData,
    // VD2_0, 3.3V
    vip_cnt_ch05: VIPData,
    // VD2_1, 3.3V
    vip_cnt_ch06: VIPData,
    // VD2_2, 3.3V
    vip_cnt_ch07: VIPData,
    // VD2_3, 3.3V
    vip_cnt_ch08: VIPData,
    // Data on conditioning chain
    ccd1: CondChnShortData,
    ccd2: CondChnShortData,
    ccd3: CondChnShortData,
    // VD0_1, 3.3V 
    vip_cnt_ch09: VIPData,
    // VD0_2, 3.3V    
    vip_cnt_ch10: VIPData,
    // VD0_3, 3.3V    
    vip_cnt_ch11: VIPData,
    // VD3_0, 5.4V (customized)
    vip_cnt_ch12: VIPData,
    // VD3_1, 5.4V (customized)
    vip_cnt_ch13: VIPData,
    // VD4_0, 12V (customized)
    vip_cnt_ch14: VIPData,
    // VD4_1, 12V (customized)
    vip_cnt_ch15: VIPData,
    // Data on conditioning chain 
    ccd4: CondChnShortData,
    ccd5: CondChnShortData,
    // Bitflag field indicating channel-on status for the extended output bus channels
    stat_ch_ext_on: u16,
    // Bitflag field indicating overcurrent latch-off fault status for the extended output bus channels
    stat_ch_ext_ocf: u16,
    // VD5_0, 28.2V (default)
    vip_cnt_ch16: VIPData,
    // Stop at 184 byte for the ICEPSv2
}
