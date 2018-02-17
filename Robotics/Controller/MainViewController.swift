//
//  MainViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class MainViewController: UIViewController, StreamDelegate, UITableViewDelegate, UITableViewDataSource {
    
    let maxReadLength = 4096
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        RequestHandler.instance.streamDelegate = self
        AppDelegate.mvc = self
        RequestHandler.instance.connectToServer()
    }

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
        
        mainView.noServerView.settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.settingsPressed)))
        
        mainView.noServerView.retryButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.retryPressed)))

        mainView.roomTableView.delegate = self
        mainView.roomTableView.dataSource = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        mainView.roomTableView.reloadData()
    }
    
    func willEnterForeground() {
        if RequestHandler.instance.paired {
            mainView.pairButton.backgroundColor =
                UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
            mainView.pairButton.layer.shadowColor =
                UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
            mainView.pairButton.setTitle("Unpair", for: .normal)
        } else {
            mainView.pairButton.backgroundColor =
                UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
            mainView.pairButton.layer.shadowColor =
                UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
            mainView.pairButton.setTitle("Pair", for: .normal)
        }
    }
    
    @objc func settingsPressed() {
        performSegue(withIdentifier: "MainToSettings", sender: self)
    }
    
    @objc func speakPressed() {
        // TODO: send request
        performSegue(withIdentifier: "MainToStatus", sender: self)
    }
    
    @objc func pairPressed() {
        if RequestHandler.instance.paired {
            RequestHandler.instance.send(command: Commands.unpair)
            mainView.pairButton.backgroundColor =
                UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
            mainView.pairButton.layer.shadowColor =
                UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
            mainView.pairButton.setTitle("Pair", for: .normal)
            RequestHandler.instance.paired = false
        } else {
            RequestHandler.instance.send(command: Commands.pair)
        }
    }
    
    @objc func retryPressed() {
        RequestHandler.instance.connectToServer()
    }
    
    // MARK: StreamDelegate
    
    func stream(_ aStream: Stream, handle eventCode: Stream.Event) {
        switch eventCode {
        case Stream.Event.hasBytesAvailable:
            print("new message received")
            let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: maxReadLength)
            let stream = aStream as! InputStream
            while stream.hasBytesAvailable {
                let numberOfBytesRead = stream.read(buffer, maxLength: 1)
                if numberOfBytesRead < 0 && stream.streamError != nil {
                    break
                }
                
                switch buffer[0] {
                case Commands.pairSucceeded:
                    print("pair succeeded")
                    mainView.pairButton.backgroundColor =
                        UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
                    mainView.pairButton.layer.shadowColor =
                        UIColor(red: 100.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
                    mainView.pairButton.setTitle("Unpair", for: .normal)
                    RequestHandler.instance.paired = true
                case Commands.pairFailed:
                    print("pair failed")
                    let alert = UIAlertController(title: "Pair failed",
                                                  message: "Someone else is using the robot. Please try again soon!",
                                                  preferredStyle: .alert)
                    
                    alert.addAction(UIAlertAction(title: "OK",
                                                  style: .default,
                                                  handler: nil))
                    
                    self.present(alert, animated: true, completion: nil)
                case Commands.gotoDone:
                    print("goto done")
                case Commands.speakDone:
                    print("speak done")
                case Commands.update:
                    print("got update")
                case Commands.unpair:
                    print("got unpair")
                    mainView.pairButton.backgroundColor =
                        UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
                    mainView.pairButton.layer.shadowColor =
                        UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
                    mainView.pairButton.setTitle("Pair", for: .normal)
                    RequestHandler.instance.paired = false
                    
                    let alert = UIAlertController(title: "Unpaired",
                                                  message: "You were unpaired with the robot due to inactivity.",
                                                  preferredStyle: .alert)
                    
                    alert.addAction(UIAlertAction(title: "OK",
                                                  style: .default,
                                                  handler: nil))
                    
                    self.present(alert, animated: true, completion: nil)
                default:
                    // Includes Commands.kill
                    print(buffer[0])
                    self.hideNoServerView(false)
                }
            }
        case Stream.Event.endEncountered:
            print("new message received")
        case Stream.Event.errorOccurred:
            print("error occurred")
            self.hideNoServerView(false)
        case Stream.Event.openCompleted:
            print("open completed")
            self.hideNoServerView(true)
        case Stream.Event.hasSpaceAvailable:
            print("has space available")
        default:
            print("some other event...")
            break
        }
    }
    
    func hideNoServerView(_ hide: Bool) {
        mainView.noServerView.isHidden = hide
        mainView.pairButton.isHidden = !hide
        mainView.speakButton.isHidden = !hide
        mainView.roomTableView.isHidden = !hide
        mainView.settingsButton.isHidden = !hide
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
        //result.selectionStyle = .none
        result.textLabel?.text = Settings.instance.roomArray[indexPath.row]
        result.accessoryType = .disclosureIndicator
        return result
    }
}

