//
//  SocketManager.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import Foundation

class SocketManager: NSObject, StreamDelegate {
    static let instance = SocketManager()
    
    var inputStream: InputStream!
    var outputStream: OutputStream!
    let maxReadLength = 4096

    override init() {
        super.init()
        self.setupNetworkCommunication()
    }
    
    func setupNetworkCommunication() {
        var readStream: Unmanaged<CFReadStream>?
        var writeStream: Unmanaged<CFWriteStream>?
        
        CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault,
                                           Settings.instance.serverIp as CFString,
                                           UInt32(Settings.instance.serverPort),
                                           &readStream,
                                           &writeStream)
        inputStream = readStream!.takeRetainedValue()
        inputStream.setProperty(StreamSocketSecurityLevel.none, forKey: Stream.PropertyKey.socketSecurityLevelKey)
        outputStream = writeStream!.takeRetainedValue()
        outputStream.setProperty(StreamSocketSecurityLevel.none, forKey: Stream.PropertyKey.socketSecurityLevelKey)
        inputStream.delegate = self
        inputStream.schedule(in: .current, forMode: .commonModes)
        outputStream.schedule(in: .current, forMode: .commonModes)
        inputStream.open()
        outputStream.open()
        //if inputStream.streamStatus == .open {
        
            let message = "test".data(using: .ascii)!
            _ = message.withUnsafeBytes { outputStream.write($0, maxLength: message.count) }
        //} else {
        //    print("bad stream")
            // TODO: make popup
        //}
    }
    
    // MARK: StreamDelegate
    
    func stream(_ aStream: Stream, handle eventCode: Stream.Event) {
        switch eventCode {
        case Stream.Event.hasBytesAvailable:
            print("new message received")
            readAvailableBytes(stream: aStream as! InputStream)
        case Stream.Event.endEncountered:
            print("new message received")
        case Stream.Event.errorOccurred:
            print("error occurred")
        case Stream.Event.hasSpaceAvailable:
            print("has space available")
        default:
            print("some other event...")
            break
        }
    }
    
    private func readAvailableBytes(stream: InputStream) {
        //1
        print("1.")
        let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: maxReadLength)
        
        //2
        while stream.hasBytesAvailable {
            //3
            let numberOfBytesRead = inputStream.read(buffer, maxLength: maxReadLength)
            
            //4
            if numberOfBytesRead < 0 {
                print("2")
                if let _ = stream.streamError {
                    break
                }
            }
            
            print("3")
            let message = String(bytesNoCopy: buffer,
                                 length: numberOfBytesRead,
                                 encoding: .ascii,
                                 freeWhenDone: true)
            
            print(message!)
        }
    }
}
