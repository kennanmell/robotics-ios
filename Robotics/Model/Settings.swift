//
//  Settings.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import Foundation

class Settings: NSObject, NSCoding {
    private static let instanceFile = FileManager().urls(
        for: .documentDirectory,
        in: .userDomainMask).first!.appendingPathComponent("arobotics.ios.settings").path
    
    private static var savedInstance = NSKeyedUnarchiver.unarchiveObject(
        withFile: instanceFile) as? Settings
    
    static var instance: Settings {
        get {
            if savedInstance == nil {
                savedInstance = Settings()
            }
            return savedInstance!
        }
    }
    
    static func save() {
        NSKeyedArchiver.archiveRootObject(Settings.instance, toFile: instanceFile)
    }

    var serverIp = "localhost"
    var serverPort = 5000
    var roomArray = Array<String>()
    var speechText = "Navigation assistance here."
    
    // MARK: NSCoding
    
    func encode(with aCoder: NSCoder) {
        aCoder.encode(serverIp, forKey: SettingsEncodingKeys.serverIpKey)
        aCoder.encode(serverPort, forKey: SettingsEncodingKeys.serverPortKey)
        aCoder.encode(roomArray.count, forKey: SettingsEncodingKeys.roomCountKey)
        aCoder.encode(speechText, forKey: SettingsEncodingKeys.speechTextKey)
        
        for i in 0..<roomArray.count {
            aCoder.encode(roomArray[i])
        }
    }
    
    required convenience init?(coder aDecoder: NSCoder) {
        self.init()
        self.serverIp =
            aDecoder.decodeObject(forKey: SettingsEncodingKeys.serverIpKey) as! String
        self.serverPort = aDecoder.decodeInteger(forKey: SettingsEncodingKeys.serverPortKey)
        self.speechText =
            aDecoder.decodeObject(forKey: SettingsEncodingKeys.speechTextKey) as! String
        
        let roomCount = aDecoder.decodeInteger(forKey: SettingsEncodingKeys.roomCountKey)
        for _ in 0..<roomCount {
            roomArray.append(aDecoder.decodeObject() as! String)
        }
    }

}

private struct SettingsEncodingKeys {
    static let serverIpKey = "com.arobotics.ios.serverIpKey"
    static let serverPortKey = "com.arobotics.ios.serverPortKey"
    static let roomCountKey = "com.arobotics.ios.roomCountKey"
    static let speechTextKey = "com.arobotics.ios.speechTextKey"
}
