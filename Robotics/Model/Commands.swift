//
//  Commands.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import Foundation

struct Commands {
    static let pair: UInt8 = 1
    static let pairSucceeded: UInt8 = 2
    static let pairFailed: UInt8 = 3
    static let unpair: UInt8 = 4
    static let speak: UInt8 = 5
    static let speakDone: UInt8 = 6
    static let goto: UInt8 = 7
    static let gotoDone: UInt8 = 8
    static let gotoFailed: UInt8 = 9
    // 10 reserved for update (not implemented)
    static let kill: UInt8 = 11
    // 12-14 reserved for robot-server pairing
    static let speakerPair: UInt8 = 15
    static let speakerPairSucceeded: UInt8 = 16
    static let speakerPairFailed: UInt8 = 17
    static let speakerUnpair: UInt8 = 18
    // 19 reserved for ping
    static let cancelGoto: UInt8 = 20
    static let cancelGotoSucceeded: UInt8 = 21
}
