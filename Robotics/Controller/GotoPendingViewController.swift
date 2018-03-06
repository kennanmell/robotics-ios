//
//  GotoPendingViewController
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class GotoPendingViewController: UIViewController {
    var pageView: PageView {
        return self.view as! PageView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.navigationItem.hidesBackButton = true
        
        pageView.textLabel.text = "Navigating to room..."
        
        let cancelButton = UIButton()
        cancelButton.setTitle("Cancel", for: .normal)
        cancelButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(GotoPendingViewController.cancelTapped)))
        pageView.addButton(button: cancelButton)
        cancelButton.backgroundColor =
            UIColor(red: 200.0 / 255.0, green: 0, blue: 0, alpha: 1.0)
        cancelButton.layer.shadowColor =
            UIColor(red: 100.0 / 255.0, green: 0, blue: 0, alpha: 1.0).cgColor
    }
    
    @objc func cancelTapped() {
        RequestHandler.instance.send(command: Commands.cancelGoto)
        self.navigationController?.popViewController(animated: true)
    }
}

