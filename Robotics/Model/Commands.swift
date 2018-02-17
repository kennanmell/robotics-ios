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
    static let gotoFailed = 9
    static let update: UInt8 = 10
    static let kill: UInt8 = 11
}
