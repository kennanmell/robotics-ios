//
//  FindMePendingViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/4/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class FindMePendingViewController: UIViewController {
    var findMeView: FindMeView {
        return self.view as! FindMeView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.navigationItem.hidesBackButton = true
        
        findMeView.textLabel.text = "Looking for you..."
        
        let cancelButton = UIButton()
        cancelButton.setTitle("Cancel", for: .normal)
        cancelButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(FindMePendingViewController.cancelTapped)))
        findMeView.addButton(button: cancelButton)
        cancelButton.backgroundColor =
            UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
        cancelButton.layer.shadowColor =
            UIColor(red: 100.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
    }
    
    @objc func cancelTapped() {
        RequestHandler.instance.send(command: Commands.cancelFindMe)
        self.navigationController?.popViewController(animated: true)
    }
}
