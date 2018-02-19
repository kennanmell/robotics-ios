//
//  RequestHandler.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import Foundation

class RequestHandler {
    static let instance = RequestHandler()

    var inputStream: InputStream!
    var outputStream: OutputStream!
    weak var streamDelegate: StreamDelegate?
    var paired = false
        
    func connectToServer() {
        if inputStream != nil {
            inputStream.close()
        }
        if outputStream != nil {
            outputStream.close()
        }
        var readStream: Unmanaged<CFReadStream>?
        var writeStream: Unmanaged<CFWriteStream>?
        
        CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault,
                                           Settings.instance.serverIp as CFString,
                                           UInt32(Settings.instance.serverPort),
                                           &readStream,
                                           &writeStream)
        inputStream = readStream!.takeRetainedValue()
        //inputStream.setProperty(StreamSocketSecurityLevel.none, forKey: Stream.PropertyKey.socketSecurityLevelKey)
        outputStream = writeStream!.takeRetainedValue()
        //outputStream.setProperty(StreamSocketSecurityLevel.none, forKey: Stream.PropertyKey.socketSecurityLevelKey)
        inputStream.delegate = self.streamDelegate
        inputStream.schedule(in: .current, forMode: .commonModes)
        outputStream.schedule(in: .current, forMode: .commonModes)
        inputStream.open()
        outputStream.open()
        
        //var message = Data()
        //message.append(self.instanceId0)
        //message.append(self.instanceId1)
        //message.append(Commands.pair)
        //let message = "test".data(using: .ascii)!
        //self.sendData(message)
        //self.send(command: Commands.pair)
    }
    
    func sendGoto(room: String) {
        var data = Data()
        data.append(Commands.goto)
        let length = room.lengthOfBytes(using: .ascii)
        data.append(UInt8(length >> 24))
        data.append(UInt8(length >> 16))
        data.append(UInt8(length >> 8))
        data.append(UInt8(length))
        data.append(room.data(using: .ascii)!)
        
        self.sendData(data)
    }
    
    func send(command: UInt8) {
        var data = Data()
        data.append(command)
        
        switch command {
        case Commands.pair: break
        case Commands.speak: break
        case Commands.unpair: break
        case Commands.kill: break
        case Commands.cancelGoto: break
        case Commands.speakerPair: break
        case Commands.speakerUnpair: break
        default: fatalError("Tried to send unsupported command to server.")
        }
        
        self.sendData(data)
    }
    
    private func sendData(_ data: Data) {
        DispatchQueue.global().async() {
            _ = data.withUnsafeBytes { self.outputStream.write($0, maxLength: data.count) }
        }
    }
}
