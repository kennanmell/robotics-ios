//
//  IntroViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit
import AVFoundation

var globalNavGoal: String? = nil

class IntroViewController: UIViewController, StreamDelegate, AVSpeechSynthesizerDelegate {
    let maxReadLength = 2
    var synthesizer: AVSpeechSynthesizer? = nil
    var isSpeaking = false
    let noServerView = NoServerView()

    var pageView: PageView {
        return self.view as! PageView
    }
    
    var connectedToServer: Bool {
        return self.noServerView.isHidden
    }
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        RequestHandler.instance.streamDelegate = self
        AppDelegate.mvc = self
        RequestHandler.instance.connectToServer()
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        pageView.textLabel.text =
            "This app controls a robot that helps you get where you need to go!"
        
        let startButton = UIButton()
        startButton.setTitle("Get Started", for: .normal)
        startButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.startTapped)))
        pageView.addButton(button: startButton)
        
        let settingsButton = UIButton()
        settingsButton.setTitle("Settings", for: .normal)
        settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.settingsTapped)))
        pageView.addButton(button: settingsButton)
        
        self.noServerView.settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.settingsTapped)))
        
        self.noServerView.retryButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(IntroViewController.retryTapped)))
        pageView.addSubview(self.noServerView)
        self.noServerView.isHidden = true
        self.noServerView.frame = self.view.frame
    }
    
    func didEnterBackground() {
        if self.navigationController?.visibleViewController is GotoPendingViewController {
            RequestHandler.instance.send(command: Commands.cancelGoto)
        } else if self.navigationController?.visibleViewController is SpeakerViewController {
            RequestHandler.instance.send(command: Commands.speakerUnpair)
        }
        
        self.navigationController?.popToRootViewController(animated: false)
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
                    let alert = UIAlertController(title: "Pair failed",
                                                  message: "Someone else is using the robot. Please try again soon!",
                                                  preferredStyle: .alert)
                    
                    alert.addAction(UIAlertAction(title: "OK",
                                                  style: .default,
                                                  handler: nil))
                    
                    self.present(alert, animated: true, completion: nil)
                case Commands.gotoDone:
                    globalNavGoal = nil
                    print("goto done")
                    if self.navigationController?.visibleViewController is GotoPendingViewController {
                        let alert = UIAlertController(title: "You have arrived.",
                                                      message: nil,
                                                      preferredStyle: .alert)
                        
                        alert.addAction(UIAlertAction(title: "OK",
                                                      style: .default,
                                                      handler: { _ in
                            self.navigationController?.popViewController(animated: false)
                            (self.navigationController?.visibleViewController as! GotoViewController).performSegue(withIdentifier: "GotoToArrived", sender: self)
                        }))
                        
                        self.present(alert, animated: true, completion: nil)
                    }
                case Commands.gotoFailed:
                    print ("goto failed")
                    globalNavGoal = nil
                    if self.navigationController?.visibleViewController is GotoPendingViewController {
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
                case Commands.unpair:
                    print("got unpair")
                    RequestHandler.instance.paired = false
                    self.navigationController?.popToRootViewController(animated: true)

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
                    
                case Commands.findMeSucceeded:
                    print("find me done")
                    if self.navigationController?.visibleViewController is FindMePendingViewController {
                        let alert = UIAlertController(title: "Found you!",
                                                      message: "The robot is right in front of you.",
                                                      preferredStyle: .alert)
                        
                        alert.addAction(UIAlertAction(title: "OK",
                                                      style: .default,
                                                      handler: { _ in
                            self.navigationController?.popViewController(animated: true)
                        }))
                        
                        self.present(alert, animated: true, completion: nil)
                    }
                case Commands.findMeFailed:
                    print ("find me failed")
                    if self.navigationController?.visibleViewController is FindMePendingViewController {
                        let alert = UIAlertController(title: "Can't find you",
                                                      message: "Make sure you're facing the robot and try again.",
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
                    self.navigationController?.popToRootViewController(animated: false)
                    self.hideNoServerView(false)
                    RequestHandler.instance.paired = false
                }
            }
        case Stream.Event.errorOccurred:
            print("error occurred")
            self.navigationController?.popToRootViewController(animated: false)
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
        self.noServerView.isHidden = hide
        self.pageView.textLabel.isHidden = !hide
        for button in self.pageView.buttonArray {
            button.isHidden = !hide
        }
    }
    
    // MARK: AVSpeechSynthesizerDelegate
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didStart utterance: AVSpeechUtterance) {
        self.isSpeaking = true
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .view.backgroundColor = UIColor.blue
        }
    }
    
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didFinish utterance: AVSpeechUtterance) {
        self.isSpeaking = false
        if navigationController?.visibleViewController is SpeakerViewController {
            (navigationController?.visibleViewController as! SpeakerViewController)
                .view.backgroundColor = UIColor.white
        }
    }
}
