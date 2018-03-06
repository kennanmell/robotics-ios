//
//  SpeakViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class SpeakViewController: UIViewController {
    var pageView: PageView {
        return self.view as! PageView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        pageView.textLabel.text = """
            Press the speak button to hear where the robot is.
            Try to get close to it and face towards it.
        """
        
        let speakButton = UIButton()
        speakButton.setTitle("Speak", for: .normal)
        speakButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakViewController.speakTapped)))
        pageView.addButton(button: speakButton)
        
        let nextButton = UIButton()
        nextButton.setTitle("Next Step", for: .normal)
        nextButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakViewController.nextTapped)))
        pageView.addButton(button: nextButton)
    }
    
    override func viewWillDisappear(_ animated : Bool) {
        super.viewWillDisappear(animated)
        
        if self.isMovingFromParentViewController {
            RequestHandler.instance.send(command: Commands.unpair)
        }
    }
    
    @objc func speakTapped() {
        RequestHandler.instance.send(command: Commands.speak)
    }
    
    @objc func nextTapped() {
        performSegue(withIdentifier: "SpeakToFindMe", sender: self)
    }
}
