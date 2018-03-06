//
//  SpeakerViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/18/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class SpeakerViewController: UIViewController {
    var pageView: PageView {
        return self.view as! PageView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationItem.hidesBackButton = true
        
        pageView.textLabel.text = Settings.instance.speechText
        pageView.textLabel.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakerViewController.textPressed)))
        pageView.textLabel.isUserInteractionEnabled = true
        
        let cancelButton = UIButton()
        cancelButton.setTitle("Cancel", for: .normal)
        cancelButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakerViewController.cancelTapped)))
        pageView.addButton(button: cancelButton)
        cancelButton.backgroundColor =
            UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
        cancelButton.layer.shadowColor =
            UIColor(red: 100.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
    }
    
    @objc func cancelTapped() {
        RequestHandler.instance.send(command: Commands.speakerUnpair)
        self.navigationController?.popViewController(animated: true)
    }
    
    @objc func textPressed() {
        let alert = UIAlertController(title: "Update speech.",
                                      message: nil,
                                      preferredStyle: .alert)
        
        alert.addTextField { (textField) in
            textField.text = Settings.instance.speechText
        }
        
        alert.addAction(UIAlertAction(title: "OK",
                                      style: .default,
                                      handler: { [weak alert] (_) in
            let newVal = alert?.textFields![0].text
            Settings.instance.speechText = newVal!
                                        self.pageView.textLabel.text = newVal
        }))
        
        self.present(alert, animated: true, completion: nil)
    }
}
