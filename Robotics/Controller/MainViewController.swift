//
//  MainViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class MainViewController: UIViewController, StreamDelegate, UITableViewDelegate, UITableViewDataSource {
    
    var inputStream: InputStream!
    var outputStream: OutputStream!
    let maxReadLength = 4096

    var mainView: MainView {
        return self.view as! MainView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()

        mainView.settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.settingsPressed)))
        
        mainView.speakButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.speakPressed)))

        mainView.pairButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.pairPressed)))
        
        mainView.roomTableView.delegate = self
        mainView.roomTableView.dataSource = self
        
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
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        mainView.roomTableView.reloadData()
    }
    
    @objc func settingsPressed() {
        performSegue(withIdentifier: "MainToSettings", sender: self)
    }
    
    @objc func speakPressed() {
        // TODO: send request
        performSegue(withIdentifier: "MainToStatus", sender: self)
    }
    
    @objc func pairPressed() {
        // TODO: send request
        performSegue(withIdentifier: "MainToStatus", sender: self)
    }
    
    // MARK: UITableViewDelegate
    
    // MARK: UITableViewDataSource
    
    func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    func tableView(_ tableView: UITableView,
                   titleForHeaderInSection section: Int) -> String? {
        return "Go To Room"
    }
    
    func tableView(_ tableView: UITableView,
                   numberOfRowsInSection section: Int) -> Int {
        return Settings.instance.roomArray.count
    }
    
    func tableView(_ tableView: UITableView,
                   cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let result = UITableViewCell()
        result.textLabel?.text = Settings.instance.roomArray[indexPath.row]
        result.accessoryType = .disclosureIndicator
        return result
    }
}

