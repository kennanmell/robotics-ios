//
//  SpeakerViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/18/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class SpeakerViewController: UIViewController {
    var speakerView: SpeakerView {
        return self.view as! SpeakerView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationItem.hidesBackButton = true
        
        speakerView.cancelButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakerViewController.cancelPressed)))

        speakerView.speechLabel.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(SpeakerViewController.textPressed)))
    }
    
    @objc func cancelPressed() {
        self.navigationController?.popViewController(animated: true)
        RequestHandler.instance.send(command: Commands.speakerUnpair)
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
            self.speakerView.speechLabel.text = newVal
        }))
        
        self.present(alert, animated: true, completion: nil)
    }
}
