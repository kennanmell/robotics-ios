//
//  ArrivedViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/4/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class ArrivedViewController: UIViewController {
    var pageView: PageView {
        return self.view as! PageView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationItem.hidesBackButton = true
        
        pageView.textLabel.text = "You have arrived!"
        
        let newDestButton = UIButton()
        newDestButton.setTitle("New Destination", for: .normal)
        newDestButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(ArrivedViewController.newDestTapped)))
        pageView.addButton(button: newDestButton)
        
        let finishButton = UIButton()
        finishButton.setTitle("Finish", for: .normal)
        finishButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(ArrivedViewController.finishTapped)))
        pageView.addButton(button: finishButton)
    }
    
    @objc func newDestTapped() {
        self.navigationController?.popViewController(animated: true)
    }
    
    @objc func finishTapped() {
        RequestHandler.instance.send(command: Commands.unpair)
        self.navigationController?.popToRootViewController(animated: true)
    }
}
