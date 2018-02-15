//
//  ViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class ViewController: UIViewController {
    
    var button = UIButton()
    
    var inputStream: InputStream!
    var outputStream: OutputStream!
    let maxReadLength = 4096

    override func viewDidLoad() {
        super.viewDidLoad()
        setupNetworkCommunication()
        // Do any additional setup after loading the view, typically from a nib.
        self.view.addSubview(button)
        button.frame = CGRect(x: self.view.frame.width / 4,
                              y: self.view.frame.height / 2 - self.view.frame.width / 8,
                              width: self.view.frame.width / 2,
                              height: self.view.frame.width / 4)
        button.backgroundColor = UIColor.blue
        button.setTitle("Button", for: .normal)
        button.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(ViewController.buttonPressed)))
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    @objc func buttonPressed() {
        print("Button pressed!")
    }
    
    func setupNetworkCommunication() {
        // 1
        var readStream: Unmanaged<CFReadStream>?
        var writeStream: Unmanaged<CFWriteStream>?
        
        // 2
        CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault,
                                           "localhost" as CFString,
                                           10000,
                                           &readStream,
                                           &writeStream)
        
        inputStream = readStream!.takeRetainedValue()
        outputStream = writeStream!.takeRetainedValue()
        
        inputStream.schedule(in: .current, forMode: .commonModes)
        outputStream.schedule(in: .current, forMode: .commonModes)

        inputStream.open()
        outputStream.open()
    }
}

