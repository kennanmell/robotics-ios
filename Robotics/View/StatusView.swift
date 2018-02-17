//
//  StatusView.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class StatusView: UIView {
    let cancelButton = UIButton()
    let pendingLabel = UILabel()
    
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
        
        self.pendingLabel.text = "Waiting for response to pair request."
        self.pendingLabel.textAlignment = .center
        self.pendingLabel.font = UIFont(name: self.pendingLabel.font.fontName, size: 20)
        self.pendingLabel.numberOfLines = 0
        self.addSubview(pendingLabel)
    }
    
    override func layoutSubviews() {
        self.pendingLabel.frame = CGRect(x: self.frame.width * 0.1,
                                         y: self.frame.width * 0.3,
                                         width: self.frame.width * 0.8,
                                         height: self.frame.height * 0.75 - self.frame.width * 0.3)

        self.cancelButton.frame = CGRect(x: self.frame.width * 0.1,
                                         y: self.frame.height * 0.8,
                                         width: self.frame.width * 0.8,
                                         height: self.frame.width * 0.15)
    }
}
