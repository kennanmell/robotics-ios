//
//  MainView.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class MainView: UIView {
    let pairButton = UIButton()
    let speakButton = UIButton()
    let roomTableView = UITableView()
    let settingsButton = UIButton()
    let noServerView = NoServerView()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        
        self.pairButton.backgroundColor =
            UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
        self.pairButton.showsTouchWhenHighlighted = true
        self.pairButton.layer.cornerRadius = 5.0
        self.pairButton.layer.borderWidth = 3.0
        self.pairButton.layer.borderColor = UIColor.clear.cgColor
        self.pairButton.layer.shadowColor =
            UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
        self.pairButton.layer.shadowOpacity = 1.0
        self.pairButton.layer.shadowRadius = 1.0
        self.pairButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        self.pairButton.setTitle("Pair", for: .normal)
        self.addSubview(pairButton)
        
        self.settingsButton.backgroundColor =
            UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
        self.settingsButton.showsTouchWhenHighlighted = true
        self.settingsButton.layer.cornerRadius = 5.0
        self.settingsButton.layer.borderWidth = 3.0
        self.settingsButton.layer.borderColor = UIColor.clear.cgColor
        self.settingsButton.layer.shadowColor =
            UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
        self.settingsButton.layer.shadowOpacity = 1.0
        self.settingsButton.layer.shadowRadius = 1.0
        self.settingsButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        self.settingsButton.setTitle("Settings", for: .normal)
        self.addSubview(settingsButton)
        
        self.speakButton.backgroundColor =
            UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
        self.speakButton.showsTouchWhenHighlighted = true
        self.speakButton.layer.cornerRadius = 5.0
        self.speakButton.layer.borderWidth = 3.0
        self.speakButton.layer.borderColor = UIColor.clear.cgColor
        self.speakButton.layer.shadowColor =
            UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
        self.speakButton.layer.shadowOpacity = 1.0
        self.speakButton.layer.shadowRadius = 1.0
        self.speakButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        self.speakButton.setTitle("Speak", for: .normal)
        self.addSubview(speakButton)
        
        roomTableView.layer.cornerRadius = 5.0
        roomTableView.layer.borderWidth = 3.0
        roomTableView.layer.borderColor = UIColor.darkGray.cgColor
        self.addSubview(roomTableView)
        
        self.noServerView.isHidden = true
        self.addSubview(noServerView)
    }
    
    override func layoutSubviews() {
        pairButton.frame = CGRect(x: self.frame.width * 0.1,
                                  y: self.frame.width * 0.3,
                                  width: self.frame.width * 0.8,
                                  height: self.frame.width * 0.15)
        
        speakButton.frame = CGRect(x: self.frame.width * 0.1,
                                   y: self.frame.width * 0.5,
                                   width: self.frame.width * 0.8,
                                   height: self.frame.width * 0.15)

        settingsButton.frame = CGRect(x: self.frame.width * 0.1,
                                      y: self.frame.width * 0.7,
                                      width: self.frame.width * 0.8,
                                      height: self.frame.width * 0.15)
        
        roomTableView.frame = CGRect(x: self.frame.width * 0.1,
                                     y: self.frame.width * 0.9,
                                     width: self.frame.width * 0.8,
                                     height: self.frame.height * 0.95 - self.frame.width * 0.9)
        
        noServerView.frame = self.frame
    }
}
