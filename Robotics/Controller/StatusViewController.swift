//
//  StatusViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class StatusViewController: UIViewController {
    var statusView: StatusView {
        return self.view as! StatusView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationItem.hidesBackButton = true
        
        statusView.cancelButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(StatusViewController.cancelPressed)))
    }
    
    @objc func cancelPressed() {
        if RequestHandler.instance.paired {
            RequestHandler.instance.send(command: Commands.cancelGoto)
        }
        self.navigationController?.popViewController(animated: true)
    }
}
