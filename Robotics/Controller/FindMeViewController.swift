//
//  FindMeViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class FindMeViewController: UIViewController {
    var pageView: PageView {
        return self.view as! PageView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        pageView.textLabel.text = """
            Press the find button then face your phone screen away from you to call the robot. You'll get an alert when it arrives.
        """
        
        let findMeButton = UIButton()
        findMeButton.setTitle("Find Me", for: .normal)
        findMeButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(FindMeViewController.findMeTapped)))
        pageView.addButton(button: findMeButton)
        
        let nextButton = UIButton()
        nextButton.setTitle("Next Step", for: .normal)
        nextButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(FindMeViewController.nextTapped)))
        pageView.addButton(button: nextButton)
    }
    
    @objc func findMeTapped() {
        RequestHandler.instance.send(command: Commands.findMe)
        performSegue(withIdentifier: "FindMeToPending", sender: self)
    }
    
    @objc func nextTapped() {
        performSegue(withIdentifier: "FindToGoto", sender: self)
    }
}
