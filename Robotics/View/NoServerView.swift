//
//  NoServerView.swift
//  Robotics
//
//  Created by Kennan Mell on 2/16/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class NoServerView: UIView {
    let retryButton = UIButton()
    let settingsButton = UIButton()
    let connectionImage = UIImageView(image: UIImage(named: "no_connection"))
    let connectionLabel = UILabel()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        self.initialize()
    }
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        self.initialize()
    }
    
    private func initialize() {
        self.backgroundColor = UIColor.white
        
        self.retryButton.backgroundColor =
            UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
        self.retryButton.showsTouchWhenHighlighted = true
        self.retryButton.layer.cornerRadius = 5.0
        self.retryButton.layer.borderWidth = 3.0
        self.retryButton.layer.borderColor = UIColor.clear.cgColor
        self.retryButton.layer.shadowColor =
            UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
        self.retryButton.layer.shadowOpacity = 1.0
        self.retryButton.layer.shadowRadius = 1.0
        self.retryButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        self.retryButton.setTitle("Retry", for: .normal)
        self.addSubview(retryButton)

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
        
        connectionImage.accessibilityLabel = "No connection"
        self.addSubview(connectionImage)
        
        connectionLabel.text = "Could not connect to server."
        self.connectionLabel.textAlignment = .center
        self.connectionLabel.font = UIFont(name: self.connectionLabel.font.fontName, size: 20)
        self.connectionLabel.numberOfLines = 0
        self.addSubview(connectionLabel)
    }
    
    override func layoutSubviews() {
        self.connectionImage.frame = CGRect(x: self.frame.width * 0.2,
                                            y: self.frame.height * 0.5 - self.frame.width * 0.3,
                                            width: self.frame.width * 0.6,
                                            height: self.frame.width * 0.6)

        self.connectionLabel.frame = CGRect(x: self.frame.width * 0.1,
                                            y: self.frame.width * 0.3,
                                            width: self.frame.width * 0.8,
                                            height: self.frame.height * 0.5 -
                                                self.frame.width * 0.6)
        
        self.retryButton.frame = CGRect(x: self.frame.width * 0.1,
                                   y: self.frame.height * 0.925 - self.frame.width * 0.3,
                                   width: self.frame.width * 0.8,
                                   height: self.frame.width * 0.15)
        
        self.settingsButton.frame = CGRect(x: self.frame.width * 0.1,
                                           y: self.frame.height * 0.95 - self.frame.width * 0.15,
                                           width: self.frame.width * 0.8,
                                           height: self.frame.width * 0.15)
    }
}
