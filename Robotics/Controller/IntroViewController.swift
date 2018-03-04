//
//  IntroViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit
import AVFoundation

class IntroViewController: UIViewController, StreamDelegate, AVSpeechSynthesizerDelegate {
    let maxReadLength = 2
    var synthesizer: AVSpeechSynthesizer? = nil
    var isSpeaking = false

    var introView: IntroView {
        return self.view as! IntroView
    }
    
    var connectedToServer: Bool {
        return introView.noServerView.isHidden
    }
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        RequestHandler.instance.streamDelegate = self
        AppDelegate.mvc = self
        RequestHandler.instance.connectToServer()
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        introView.textLabel.text =
            "This app controls a robot that helps you get where you need to go!"
        
        let startButton = UIButton()
        startButton.setTitle("Get Started", for: .normal)
        startButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.startTapped)))
        introView.addButton(button: startButton)
        
        let settingsButton = UIButton()
        settingsButton.setTitle("Settings", for: .normal)
        settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.settingsTapped)))
        introView.addButton(button: settingsButton)
        
        introView.noServerView.settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.settingsTapped)))
        
        introView.noServerView.retryButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.retryTapped)))
    }
    
    func didEnterBackground() {
        if self.navigationController?.visibleViewController is StatusViewController {
            RequestHandler.instance.send(command: Commands.cancelGoto)
            self.navigationController?.popViewController(animated: true)
        } else if self.navigationController?.visibleViewController is SpeakerViewController {
            RequestHandler.instance.send(command: Commands.speakerUnpair)
            self.navigationController?.popViewController(animated: true)
        }
    }
    
    // MARK: Callbacks
    
    @objc func startTapped() {
        RequestHandler.instance.sendPair()
    }
    
    @objc func settingsTapped() {
        performSegue(withIdentifier: "IntroToSettings", sender: self)
    }
    
    @objc func retryTapped() {
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
                    RequestHandler.instance.paired = true
                    if self.navigationController?.visibleViewController is IntroViewController {
                        performSegue(withIdentifier: "IntroToSpeak", sender: self)
                    }
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
                    if self.navigationController?.visibleViewController is SpeakerViewController && !self.isSpeaking {
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
                    RequestHandler.instance.paired = false
                    while !(self.navigationController?.visibleViewController is IntroViewController) {
                        self.navigationController?.popViewController(animated: false)
                    }
                    
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
                    while !(self.navigationController?.visibleViewController is IntroViewController) {
                        self.navigationController?.popViewController(animated: false)
                    }
                    self.hideNoServerView(false)
                    RequestHandler.instance.paired = false
                }
            }
        case Stream.Event.errorOccurred:
            print("error occurred")
            while !(self.navigationController?.visibleViewController is IntroViewController) {
                self.navigationController?.popViewController(animated: false)
            }
            self.hideNoServerView(false)
            RequestHandler.instance.paired = false
        case Stream.Event.openCompleted:
            print("open completed")
            self.hideNoServerView(true)
        default:
            print("unhandled event...")
            break
        }
    }
    
    func hideNoServerView(_ hide: Bool) {
        self.introView.noServerView.isHidden = hide
        self.introView.textLabel.isHidden = !hide
        for button in self.introView.buttonArray {
            button.isHidden = !hide
        }
    }
    
    // MARK: AVSpeechSynthesizerDelegate
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didStart utterance: AVSpeechUtterance) {
        self.isSpeaking = true
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .speakerView.backgroundColor = UIColor.blue
        }
    }
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didFinish utterance: AVSpeechUtterance) {
        self.isSpeaking = false
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .speakerView.backgroundColor = UIColor.white
        }
    }
}
