//
//  MainViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit
import AVFoundation

var globalNavGoal: String? = nil

class MainViewController: UIViewController, StreamDelegate, AVSpeechSynthesizerDelegate,
                          UITableViewDelegate, UITableViewDataSource {
    
    let maxReadLength = 2
    var synthesizer: AVSpeechSynthesizer? = nil
    
    var mainView: MainView {
        return self.view as! MainView
    }
    
    var connectedToServer: Bool {
        return mainView.noServerView.isHidden
    }
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        RequestHandler.instance.streamDelegate = self
        AppDelegate.mvc = self
        RequestHandler.instance.connectToServer()
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
        willEnterForeground()
    }
    
    func didEnterBackground() {
        if self.navigationController?.visibleViewController is StatusViewController {
            RequestHandler.instance.send(command: Commands.cancelGoto)
            self.navigationController?.popViewController(animated: true)
        }
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
    
    // MARK: Callbacks
    
    @objc func settingsPressed() {
        performSegue(withIdentifier: "MainToSettings", sender: self)
    }
    
    @objc func speakPressed() {
        if !RequestHandler.instance.paired {
            RequestHandler.instance.send(command: Commands.pair)
        }
        RequestHandler.instance.send(command: Commands.speak)
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
        willEnterForeground()
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
                    globalNavGoal = nil
                    let alert = UIAlertController(title: "Pair failed",
                                                  message: "Someone else is using the robot. Please try again soon!",
                                                  preferredStyle: .alert)
                    
                    alert.addAction(UIAlertAction(title: "OK",
                                                  style: .default,
                                                  handler: { _ in
                        if self.navigationController?.visibleViewController is StatusViewController {
                            self.navigationController?.popViewController(animated: true)
                        }
                    }))
                    
                    self.present(alert, animated: true, completion: nil)
                case Commands.gotoDone:
                    globalNavGoal = nil
                    print("goto done")
                    if self.navigationController?.visibleViewController is StatusViewController {
                        let alert = UIAlertController(title: "You have arrived.",
                                                      message: nil,
                                                      preferredStyle: .alert)
                    
                        alert.addAction(UIAlertAction(title: "OK",
                                                      style: .default,
                                                      handler: { _ in
                            self.navigationController?.popViewController(animated: true)
                        }))
                    
                        self.present(alert, animated: true, completion: nil)
                    }
                case Commands.gotoFailed:
                    print ("goto failed")
                    globalNavGoal = nil
                    if self.navigationController?.visibleViewController is StatusViewController {
                        let alert = UIAlertController(title: "Navigation failed",
                                                      message: "Couldn't find a path to the room.",
                                                      preferredStyle: .alert)
                        
                        alert.addAction(UIAlertAction(title: "OK",
                                                      style: .default,
                                                      handler: { _ in
                            self.navigationController?.popViewController(animated: true)
                        }))
                        
                        self.present(alert, animated: true, completion: nil)
                    }
                case Commands.speakerSpeak:
                    print("got speaker speak")
                    if self.navigationController?.visibleViewController is SpeakerViewController {
                        let utterance = AVSpeechUtterance(string: Settings.instance.speechText)
                        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
                        self.synthesizer = AVSpeechSynthesizer()
                        self.synthesizer?.delegate = self
                        self.synthesizer?.speak(utterance)
                    }
                case Commands.speakFailed:
                    print("got speak failed")
                    let alert = UIAlertController(title: "Speak failed",
                                                message: "No speaker is connected.",
                                         preferredStyle: .alert)
                        
                    alert.addAction(UIAlertAction(title: "OK",
                                                  style: .default,
                                                handler: nil))
                        
                    self.present(alert, animated: true, completion: nil)
                case Commands.cancelGotoSucceeded:
                    print("got cancel succeeded")
                    if self.navigationController?.visibleViewController is StatusViewController {
                        self.navigationController?.popViewController(animated: true)
                    }
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
                case Commands.speakerPairSucceeded:
                    print("got speaker pair succeeded")
                case Commands.speakerPairFailed:
                    print("got speaker pair failed")
                    if self.navigationController?.visibleViewController is SpeakerViewController {
                        let alert = UIAlertController(title: "Pair failed",
                                                    message: "Another speaker is already connected.",
                                             preferredStyle: .alert)
                        
                        alert.addAction(UIAlertAction(title: "OK",
                                                      style: .default,
                                                    handler: { _ in
                            self.navigationController?.popViewController(animated: true)
                        }))
                        
                        self.present(alert, animated: true, completion: nil)
                    }
                default:
                    // Includes Commands.kill
                    print(buffer[0])
                    self.hideNoServerView(false)
                    RequestHandler.instance.paired = false
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
    
    // MARK: AVSpeechSynthesizerDelegate
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didStart utterance: AVSpeechUtterance) {
        print("in delegate start")
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .speakerView.backgroundColor = UIColor.blue
        }
    }
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didFinish utterance: AVSpeechUtterance) {
        print("in delegate end")
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .speakerView.backgroundColor = UIColor.white
        }
    }
    
    // MARK: UITableViewDelegate
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        performSegue(withIdentifier: "MainToStatus", sender: self)
        if !RequestHandler.instance.paired {
            RequestHandler.instance.send(command: Commands.pair)
        }
        globalNavGoal = Settings.instance.roomArray[indexPath.row]
        RequestHandler.instance.sendGoto(room: Settings.instance.roomArray[indexPath.row])
    }
    
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

