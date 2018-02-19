//
//  SpeakerView.swift
//  Robotics
//
//  Created by Kennan Mell on 2/18/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class SpeakerView: UIView {
    let speechLabel = UILabel()
    let cancelButton = UIButton()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        
        self.cancelButton.backgroundColor =
            UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
        self.cancelButton.showsTouchWhenHighlighted = true
        self.cancelButton.layer.cornerRadius = 5.0
        self.cancelButton.layer.borderWidth = 3.0
        self.cancelButton.layer.borderColor = UIColor.clear.cgColor
        self.cancelButton.layer.shadowColor =
            UIColor(red: 100.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
        self.cancelButton.layer.shadowOpacity = 1.0
        self.cancelButton.layer.shadowRadius = 1.0
        self.cancelButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        self.cancelButton.setTitle("Cancel", for: .normal)
        self.addSubview(cancelButton)
        
        self.speechLabel.text = Settings.instance.speechText
        self.speechLabel.textAlignment = .center
        self.speechLabel.font = UIFont(name: self.speechLabel.font.fontName, size: 20)
        self.speechLabel.numberOfLines = 0
        self.speechLabel.isUserInteractionEnabled = true
        self.addSubview(speechLabel)
    }
    
    override func layoutSubviews() {
        self.speechLabel.frame = CGRect(x: self.frame.width * 0.1,
                                        y: self.frame.width * 0.3,
                                        width: self.frame.width * 0.8,
                                        height: self.frame.height - self.frame.width * 0.6)

        self.cancelButton.frame = CGRect(x: self.frame.width * 0.1,
                                         y: self.frame.height * 0.95 - self.frame.width * 0.25,
                                         width: self.frame.width * 0.8,
                                         height: self.frame.width * 0.15)
    }
}
